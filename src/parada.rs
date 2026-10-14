//! Parada segura: el proceso nunca termina ni se queda callado dejando a un robot con el
//! último comando. La base station no tiene timeout de serial: si el PC deja de mandar,
//! repite el último frame para siempre, y el watchdog del robot (200 ms) solo cubre la
//! pérdida de radio.
//!
//! - [`mandar_ceros`]: la única ruta de ceros (cuántos frames y cada cuánto), para todos
//!   los binarios. Los comandos en cero del lazo son los de HALT
//!   (`control_loop::parada_del_lazo`).
//! - [`parar_al_terminar`]: corre el lazo y para al terminar, también si entra en pánico.
//! - [`al_recibir_senal`]: SIGINT, SIGTERM y SIGHUP activan `shutdown`.
//! - [`con_gui`]: la GUI en el hilo principal; cuando se cierra o entra en pánico, espera
//!   la parada del lazo antes de devolver el código de salida.
//!
//! Lo que el PC no puede atrapar (SIGKILL, `abort`, el OOM killer, un corte de energía, el
//! USB desconectado) depende del timeout de serial de la base, pendiente en el firmware.

use crate::radio::TransportError;
use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::pin::pin;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::task::Poll;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// Frames en cero de una parada.
pub const PARADA_FRAMES: usize = 6;
/// Entre frames: la cadencia con que la base reenvía al robot.
pub const PARADA_PERIODO: Duration = Duration::from_millis(50);
/// Tope de cada envío: uno colgado no cuelga la parada.
pub const PARADA_ENVIO_MAX: Duration = Duration::from_millis(100);
/// Cuánto espera [`con_gui`] a que el lazo termine su parada.
pub const PARADA_ESPERA_MAX: Duration = Duration::from_secs(2);

/// La parada: `PARADA_FRAMES` frames en cero, uno cada `PARADA_PERIODO` (~300 ms). La base
/// se queda con la última línea y la reenvía cada 50 ms, así que ve varias en cero aunque
/// una se pierda. `enviar` arma y manda un frame en cero por el transporte de quien llama.
/// Un error o un envío que no termina en `PARADA_ENVIO_MAX` se informan y no cortan la
/// secuencia. Devuelve cuántos frames salieron bien.
pub async fn mandar_ceros(mut enviar: impl AsyncFnMut() -> Result<(), TransportError>) -> usize {
    let mut ok = 0;
    for i in 1..=PARADA_FRAMES {
        if i > 1 {
            tokio::time::sleep(PARADA_PERIODO).await;
        }
        match tokio::time::timeout(PARADA_ENVIO_MAX, enviar()).await {
            Ok(Ok(())) => ok += 1,
            Ok(Err(e)) => eprintln!("[parada] frame {i} en cero: error al enviar: {e}"),
            Err(_) => eprintln!(
                "[parada] frame {i} en cero: el envío no terminó en {} ms",
                PARADA_ENVIO_MAX.as_millis()
            ),
        }
    }
    eprintln!("[parada] {ok} de {PARADA_FRAMES} frames en cero enviados");
    ok
}

/// `catch_unwind` para un future: corre `fut` y devuelve `Err` con el pánico si lo hubo.
/// El future se suelta antes de volver, y con él los guards que tuviera tomados.
pub async fn atrapar_panico<F: Future>(fut: F) -> std::thread::Result<F::Output> {
    let mut fut = pin!(fut);
    std::future::poll_fn(|cx| {
        match std::panic::catch_unwind(AssertUnwindSafe(|| fut.as_mut().poll(cx))) {
            Ok(Poll::Ready(v)) => Poll::Ready(Ok(v)),
            Ok(Poll::Pending) => Poll::Pending,
            Err(panico) => Poll::Ready(Err(panico)),
        }
    })
    .await
}

/// Corre `lazo` y después `parar`, también si `lazo` entra en pánico; en ese caso
/// re-lanza el pánico después de parar.
pub async fn parar_al_terminar<T>(lazo: impl Future<Output = T>, parar: impl AsyncFnOnce()) -> T {
    let fin = atrapar_panico(lazo).await;
    if fin.is_err() {
        eprintln!("[parada] pánico en el lazo: mando ceros antes de seguir");
    }
    parar().await;
    match fin {
        Ok(v) => v,
        Err(panico) => std::panic::resume_unwind(panico),
    }
}

/// Registra SIGINT (Ctrl-C), SIGTERM y SIGHUP (se cerró la terminal) y, con la primera que
/// llegue, activa `shutdown`: el lazo para con ceros antes de que termine el proceso. Los
/// manejadores quedan registrados al volver; se llama dentro de un runtime de tokio. Las
/// señales siguientes no hacen nada: la parada termina sola en menos de un segundo.
pub fn al_recibir_senal(shutdown: Arc<AtomicBool>) -> std::io::Result<()> {
    #[cfg(unix)]
    {
        use tokio::signal::unix::{SignalKind, signal};
        let mut int = signal(SignalKind::interrupt())?;
        let mut term = signal(SignalKind::terminate())?;
        let mut hup = signal(SignalKind::hangup())?;
        tokio::spawn(async move {
            let nombre = tokio::select! {
                _ = int.recv() => "SIGINT (Ctrl-C)",
                _ = term.recv() => "SIGTERM",
                _ = hup.recv() => "SIGHUP",
            };
            eprintln!("[parada] {nombre}: detengo los robots y salgo");
            shutdown.store(true, Ordering::Relaxed);
        });
    }
    // Windows (compilación y pruebas): solo Ctrl-C.
    #[cfg(not(unix))]
    tokio::spawn(async move {
        if tokio::signal::ctrl_c().await.is_ok() {
            eprintln!("[parada] Ctrl-C: detengo los robots y salgo");
            shutdown.store(true, Ordering::Relaxed);
        }
    });
    Ok(())
}

/// Corre la GUI en este hilo (el principal) con el lazo en el hilo `lazo`, y no deja
/// terminar el proceso sin parar: cuando la GUI vuelve (se cerró la ventana, o porque
/// terminó el lazo) o entra en pánico, activa `shutdown` y espera hasta
/// `PARADA_ESPERA_MAX` a que el lazo termine su parada. Devuelve el código de salida: 0;
/// 1 si la GUI devolvió error o el lazo no terminó; 101 si hubo un pánico, en la GUI o en
/// el lazo.
pub fn con_gui<E: std::fmt::Display>(
    gui: impl FnOnce() -> Result<(), E>,
    lazo: JoinHandle<()>,
    shutdown: &AtomicBool,
) -> i32 {
    let gui = std::panic::catch_unwind(AssertUnwindSafe(gui));
    shutdown.store(true, Ordering::Relaxed);
    let hasta = Instant::now() + PARADA_ESPERA_MAX;
    while !lazo.is_finished() && Instant::now() < hasta {
        std::thread::sleep(Duration::from_millis(10));
    }
    let lazo = if lazo.is_finished() {
        Some(lazo.join().is_ok())
    } else {
        eprintln!(
            "[parada] el lazo no terminó su parada en {} s",
            PARADA_ESPERA_MAX.as_secs()
        );
        None
    };
    match (gui, lazo) {
        (Err(_), _) | (_, Some(false)) => 101,
        (Ok(Err(e)), _) => {
            eprintln!("[parada] la GUI terminó con error: {e}");
            1
        }
        (Ok(Ok(())), None) => 1,
        (Ok(Ok(())), Some(true)) => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    #[tokio::test]
    async fn six_zero_frames_over_about_300_ms() {
        let envios: Mutex<Vec<Instant>> = Mutex::new(Vec::new());
        let ok = mandar_ceros(async || {
            envios.lock().unwrap().push(Instant::now());
            Ok(())
        })
        .await;
        let envios = envios.into_inner().unwrap();
        assert_eq!((ok, envios.len()), (PARADA_FRAMES, PARADA_FRAMES));
        let total = envios[PARADA_FRAMES - 1] - envios[0];
        assert!(total >= Duration::from_millis(250), "{total:?}");
    }

    #[tokio::test]
    async fn a_failed_send_does_not_cut_the_stop() {
        let mut intentos = 0;
        let ok = mandar_ceros(async || {
            intentos += 1;
            if intentos == 2 {
                Err("puerto caído".into())
            } else {
                Ok(())
            }
        })
        .await;
        assert_eq!((intentos, ok), (PARADA_FRAMES, PARADA_FRAMES - 1));
    }

    #[tokio::test]
    async fn a_hung_send_does_not_hang_the_stop() {
        let mut intentos = 0;
        let empezo = Instant::now();
        let ok = mandar_ceros(async || {
            intentos += 1;
            if intentos == 1 {
                std::future::pending::<()>().await;
            }
            Ok(())
        })
        .await;
        assert_eq!((intentos, ok), (PARADA_FRAMES, PARADA_FRAMES - 1));
        assert!(empezo.elapsed() < Duration::from_secs(1));
    }

    #[tokio::test]
    async fn stop_runs_when_the_loop_ends() {
        let paso = Mutex::new(Vec::new());
        let v = parar_al_terminar(async { 7 }, async || paso.lock().unwrap().push("parar")).await;
        assert_eq!(v, 7);
        assert_eq!(*paso.lock().unwrap(), ["parar"]);
    }

    #[tokio::test]
    async fn stop_runs_before_a_loop_panic_goes_on() {
        let paso = Mutex::new(Vec::new());
        let fin = atrapar_panico(parar_al_terminar(
            async {
                paso.lock().unwrap().push("lazo");
                panic!("pánico de prueba en el lazo");
            },
            async || paso.lock().unwrap().push("parar"),
        ))
        .await;
        assert!(fin.is_err(), "el pánico sigue después de parar");
        assert_eq!(*paso.lock().unwrap(), ["lazo", "parar"]);
    }

    /// Espera hasta 2 s a que `flag` se active.
    #[cfg(unix)]
    async fn se_activa(flag: &AtomicBool) -> bool {
        let hasta = Instant::now() + Duration::from_secs(2);
        while Instant::now() < hasta {
            if flag.load(Ordering::Relaxed) {
                return true;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        false
    }

    /// Se manda la señal a sí mismo, con los manejadores ya registrados.
    #[cfg(unix)]
    fn mandarme(senal: &str) {
        let estado = std::process::Command::new("kill")
            .args(["-s", senal, &std::process::id().to_string()])
            .status()
            .expect("kill");
        assert!(estado.success());
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn sigterm_sighup_and_sigint_trigger_the_shutdown() {
        for senal in ["TERM", "HUP", "INT"] {
            let shutdown = Arc::new(AtomicBool::new(false));
            al_recibir_senal(shutdown.clone()).unwrap();
            mandarme(senal);
            assert!(se_activa(&shutdown).await, "SIG{senal} no activó shutdown");
        }
    }

    /// Hilo de lazo de prueba: al ver `shutdown`, "para" (tarda 50 ms) y lo anota.
    fn lazo_de_prueba(
        shutdown: &Arc<AtomicBool>,
        parado: &Arc<AtomicBool>,
        panico: bool,
    ) -> JoinHandle<()> {
        let (shutdown, parado) = (shutdown.clone(), parado.clone());
        std::thread::spawn(move || {
            while !shutdown.load(Ordering::Relaxed) {
                std::thread::sleep(Duration::from_millis(1));
            }
            std::thread::sleep(Duration::from_millis(50));
            parado.store(true, Ordering::Relaxed);
            if panico {
                panic!("pánico de prueba en el hilo del lazo");
            }
        })
    }

    /// Corre `con_gui` con una GUI de prueba y devuelve el código y si el lazo ya había
    /// parado al volver.
    fn correr(gui: impl FnOnce() -> Result<(), String>, panico_del_lazo: bool) -> (i32, bool) {
        let shutdown = Arc::new(AtomicBool::new(false));
        let parado = Arc::new(AtomicBool::new(false));
        let lazo = lazo_de_prueba(&shutdown, &parado, panico_del_lazo);
        let codigo = con_gui(gui, lazo, &shutdown);
        (codigo, parado.load(Ordering::Relaxed))
    }

    #[test]
    fn closing_the_window_waits_for_the_stop() {
        assert_eq!(correr(|| Ok(()), false), (0, true));
    }

    #[test]
    fn a_gui_error_waits_for_the_stop() {
        assert_eq!(correr(|| Err("sin ventana".to_string()), false), (1, true));
    }

    #[test]
    fn a_gui_panic_waits_for_the_stop() {
        assert_eq!(
            correr(|| panic!("pánico de prueba en la GUI"), false),
            (101, true)
        );
    }

    #[test]
    fn a_loop_panic_gives_101() {
        assert_eq!(correr(|| Ok(()), true), (101, true));
    }
}
