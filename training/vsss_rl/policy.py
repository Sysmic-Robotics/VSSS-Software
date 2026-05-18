"""
Red neuronal Actor-Critic para PPO en VSSS — implementación en PyTorch puro.

Arquitectura completa (incluyendo cabezas finales agregadas por SB3):

    Observación (Box, 52) ──┬──▶ ACTOR π:   Linear(52→256) ─ Tanh
                            │                Linear(256→256) ─ Tanh
                            │                Linear(256→2)          ◀── acción μ (head)
                            │                +  log_σ (param global)
                            │
                            └──▶ CRÍTICO V: Linear(52→256) ─ Tanh
                                            Linear(256→256) ─ Tanh
                                            Linear(256→1)           ◀── V(s) (head)

Las DOS TORRES (8 Linear + 4 Tanh por torre, sin las cabezas finales) están
definidas explícitamente como subclase de `nn.Module` en este archivo, siguiendo
el patrón estándar de PyTorch que enseñan textos como:

    - Bishop, "Deep Learning: Foundations and Concepts" (2024), cap. 6
    - Murphy, "Probabilistic Machine Learning: An Introduction" (2022), cap. 13

Las cabezas finales (`action_net = Linear(256→2)` y `value_net = Linear(256→1)`)
las construye Stable-Baselines3 al instanciar `ActorCriticPolicy`. Esta es la
separación estándar en RL: el "extractor" devuelve representaciones latentes y
las cabezas pequeñas mapean a la acción y al valor.

Función de activación: Tanh, no ReLU. Razón:
- Tanh acota la salida a [-1, 1], lo que estabiliza el entrenamiento de PPO
  (gradientes mejor escalados cuando la observación ya está normalizada).
- Es la elección estándar en la literatura PPO (Schulman et al. 2017).
- ReLU en políticas de control continuo tiende a producir "neuronas muertas".

Conteo de parámetros (sin cabezas):
    Por torre: (52·256 + 256) + (256·256 + 256) = 79,104
    Dos torres: 158,208
    + log_std (2,) = 2 parámetros aprendibles globales
    + Cabezas SB3: Linear(256→2) + Linear(256→1) = (256·2+2) + (256·1+1) = 771
    Total: ~158,981 parámetros

Notas de diseño:
- Torres separadas (no compartidas) → permiten que actor y crítico aprendan
  representaciones distintas. Coste: más parámetros. Beneficio: estabilidad.
- `latent_dim_pi` y `latent_dim_vf` son atributos que SB3 lee para saber
  cuántas neuronas entregar a cada cabeza final.
"""

from __future__ import annotations

import torch
import torch.nn as nn
from stable_baselines3.common.policies import ActorCriticPolicy


# ── Constantes de arquitectura ───────────────────────────────────────────────
OBS_DIM: int = 52       # contrato Rust (src/coach/observation.rs)
HIDDEN_DIM: int = 256   # neuronas por capa oculta


class VsssActorCriticNet(nn.Module):
    """Las dos torres MLP del modelo Actor-Critic.

    Cada torre es una MLP de 2 capas ocultas con activación Tanh.
    No incluye las cabezas finales (esas las agrega SB3 automáticamente).

    Parameters
    ----------
    feature_dim : int
        Dimensión de la observación de entrada (52 para VSSS).
    hidden : int
        Neuronas por capa oculta.
    """

    def __init__(self, feature_dim: int = OBS_DIM, hidden: int = HIDDEN_DIM):
        super().__init__()

        # SB3 lee estos atributos para construir las cabezas finales.
        # latent_dim_pi: dimensión de salida de la torre del actor.
        # latent_dim_vf: dimensión de salida de la torre del crítico.
        self.latent_dim_pi: int = hidden
        self.latent_dim_vf: int = hidden

        # ── Torre del actor (política π_θ) ────────────────────────────────
        # Mapea obs ∈ ℝ^52 a una representación latente ∈ ℝ^256
        # sobre la cual SB3 montará Linear(256→2) para los parámetros μ
        # de la distribución gaussiana de la acción.
        self.actor_trunk: nn.Sequential = nn.Sequential(
            nn.Linear(feature_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
        )

        # ── Torre del crítico (función de valor V_ψ) ──────────────────────
        # Mapea obs ∈ ℝ^52 a una representación latente ∈ ℝ^256
        # sobre la cual SB3 montará Linear(256→1) para el valor V(s).
        self.critic_trunk: nn.Sequential = nn.Sequential(
            nn.Linear(feature_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
        )

    def forward(
        self, features: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Forward pass de ambas torres.

        Parameters
        ----------
        features : torch.Tensor of shape (batch, 52)
            Observaciones del batch.

        Returns
        -------
        latent_pi : torch.Tensor of shape (batch, 256)
            Latente del actor (entrada a la cabeza de acción).
        latent_vf : torch.Tensor of shape (batch, 256)
            Latente del crítico (entrada a la cabeza de valor).
        """
        return self.forward_actor(features), self.forward_critic(features)

    def forward_actor(self, features: torch.Tensor) -> torch.Tensor:
        """Forward solo del actor — usado durante `predict()` en deploy."""
        return self.actor_trunk(features)

    def forward_critic(self, features: torch.Tensor) -> torch.Tensor:
        """Forward solo del crítico — usado para estimar V(s)."""
        return self.critic_trunk(features)


class VsssActorCriticPolicy(ActorCriticPolicy):
    """Policy que reemplaza el extractor MLP por defecto de SB3 con nuestra
    `VsssActorCriticNet`.

    Esta es la clase que se pasa a `PPO(policy=VsssActorCriticPolicy, ...)`.

    SB3 hace el resto:
    1. Construye un `features_extractor` (identidad para vector input)
    2. Llama a `_build_mlp_extractor()` → instancia nuestra red
    3. Agrega las cabezas: `action_net = Linear(256, 2)`, `value_net = Linear(256, 1)`
    4. Construye la distribución gaussiana con `log_std` global aprendible
    """

    def _build_mlp_extractor(self) -> None:
        """Override del método de SB3 — instancia nuestra red propia."""
        self.mlp_extractor = VsssActorCriticNet(
            feature_dim=self.features_dim,
            hidden=HIDDEN_DIM,
        )


def count_parameters(net: nn.Module) -> int:
    """Cuenta parámetros aprendibles (útil para reportar en el informe)."""
    return sum(p.numel() for p in net.parameters() if p.requires_grad)


if __name__ == "__main__":
    # Smoke test: instanciar la red, ejecutar un forward y reportar parámetros.
    net = VsssActorCriticNet()
    print(net)
    print(f"\nParámetros totales: {count_parameters(net):,}")

    batch = torch.zeros(8, OBS_DIM)  # batch de 8 observaciones de prueba
    latent_pi, latent_vf = net(batch)
    print(f"\nlatent_pi shape: {tuple(latent_pi.shape)}  (esperado: (8, 256))")
    print(f"latent_vf shape: {tuple(latent_vf.shape)}  (esperado: (8, 256))")
