"""Genera las fuentes de la GUI: Barlow con cifras tabulares por defecto.

    python3 tools/fuentes_tabulares.py ORIGEN assets/fonts

ORIGEN tiene los TTF originales de google/fonts (`ofl/barlow` y
`ofl/barlowsemicondensed`, commit 89f5431ff0db41bd2fe3f7ba21a723a01622428b):
Barlow-Regular.ttf, Barlow-SemiBold.ttf, BarlowSemiCondensed-Medium.ttf y
BarlowSemiCondensed-SemiBold.ttf. Requiere fontTools (no es parte de la compilación:
los TTF generados se versionan).

Por qué: la GUI es iced 0.13, cuyo texto (cosmic-text 0.12) no aplica rasgos
OpenType, y las cifras por defecto de Barlow son proporcionales (en Regular, el `0`
mide 565 unidades y el `1`, 350). El script:

- apunta el `cmap` de `0`–`9` a los glifos del rasgo `tnum` (`zero.tf`, ...), así
  las cifras tabulares son las de por defecto;
- renombra la familia ("Barlow Tabular", "Barlow Semi Condensed Tabular") para que
  no se mezcle con una Barlow instalada en el sistema, que fontdb también carga;
- falla si los 10 dígitos no quedan con el mismo avance.

La licencia es la OFL 1.1, sin Reserved Font Name: permite modificar y redistribuir
con la licencia, que va en `assets/fonts/OFL.txt`. No se tocan los timestamps, así
que el resultado es el mismo en cada corrida.
"""

import os
import sys

from fontTools.ttLib import TTFont

# Original → generado, y la familia original → la nueva (el más largo primero).
FUENTES = {
    "Barlow-Regular.ttf": "BarlowTabular-Regular.ttf",
    "Barlow-SemiBold.ttf": "BarlowTabular-SemiBold.ttf",
    "BarlowSemiCondensed-Medium.ttf": "BarlowSemiCondensedTabular-Medium.ttf",
    "BarlowSemiCondensed-SemiBold.ttf": "BarlowSemiCondensedTabular-SemiBold.ttf",
}
FAMILIAS = [
    ("Barlow Semi Condensed", "Barlow Semi Condensed Tabular"),
    ("Barlow", "Barlow Tabular"),
]
POSTSCRIPT = [
    ("BarlowSemiCondensed-", "BarlowSemiCondensedTabular-"),
    ("Barlow-", "BarlowTabular-"),
]
DIGITOS = "0123456789"
# Registros de nombre que llevan la familia o el nombre PostScript.
IDS_FAMILIA = (1, 4, 16)
IDS_POSTSCRIPT = (3, 6)


def glifos_tnum(font):
    """Mapa glifo → glifo `.tf` de las sustituciones simples del rasgo `tnum`."""
    gsub = font["GSUB"].table
    lookups = set()
    for fr in gsub.FeatureList.FeatureRecord:
        if fr.FeatureTag == "tnum":
            lookups.update(fr.Feature.LookupListIndex)
    mapa = {}
    for i in lookups:
        for st in gsub.LookupList.Lookup[i].SubTable:
            st = getattr(st, "ExtSubTable", st)
            mapa.update(getattr(st, "mapping", {}))
    return mapa


def congelar_tnum(font):
    """Apunta el cmap de los dígitos a sus glifos tabulares."""
    tnum = glifos_tnum(font)
    tabulares = set(tnum.values())
    for tabla in font["cmap"].tables:
        if not tabla.isUnicode():
            continue
        for c in DIGITOS:
            glifo = tabla.cmap.get(ord(c))
            # Los subtables pueden compartir el diccionario: ya puede estar hecho.
            if glifo is None or glifo in tabulares:
                continue
            if glifo not in tnum:
                raise SystemExit(f"{c!r} ({glifo}) no tiene glifo tnum")
            tabla.cmap[ord(c)] = tnum[glifo]


def renombrar(texto, reemplazos, solo_prefijo):
    for viejo, nuevo in reemplazos:
        if solo_prefijo:
            if texto.startswith(viejo):
                return nuevo + texto[len(viejo):]
        elif viejo in texto:
            return texto.replace(viejo, nuevo, 1)
    return texto


def renombrar_familia(font):
    tabla = font["name"]
    for rec in list(tabla.names):
        texto = rec.toUnicode()
        if rec.nameID in IDS_FAMILIA:
            nuevo = renombrar(texto, FAMILIAS, solo_prefijo=True)
        elif rec.nameID in IDS_POSTSCRIPT:
            nuevo = renombrar(texto, POSTSCRIPT, solo_prefijo=False)
        else:
            continue
        if nuevo != texto:
            tabla.setName(nuevo, rec.nameID, rec.platformID, rec.platEncID, rec.langID)


def verificar(font, nombre):
    cmap = font.getBestCmap()
    avances = {font["hmtx"][cmap[ord(c)]][0] for c in DIGITOS}
    if len(avances) != 1:
        raise SystemExit(f"{nombre}: los dígitos no quedaron tabulares: {sorted(avances)}")
    familias = {r.toUnicode() for r in font["name"].names if r.nameID in (1, 16)}
    if not any("Tabular" in f for f in familias):
        raise SystemExit(f"{nombre}: la familia no quedó renombrada: {familias}")
    return avances.pop()


def main(argv):
    if len(argv) != 3:
        print(__doc__.strip().split("\n\n")[1])
        return 2
    origen, destino = argv[1], argv[2]
    os.makedirs(destino, exist_ok=True)
    for original, generado in FUENTES.items():
        font = TTFont(os.path.join(origen, original), recalcTimestamp=False)
        congelar_tnum(font)
        renombrar_familia(font)
        avance = verificar(font, generado)
        ruta = os.path.join(destino, generado)
        font.save(ruta)
        familia = sorted({r.toUnicode() for r in font["name"].names if r.nameID in (1, 16)})
        print(f"{ruta}: dígitos de {avance} unidades, familia {familia}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
