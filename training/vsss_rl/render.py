"""
Renderizador del mini-sim con matplotlib.

Dibuja el campo VSSS, el robot azul (círculo + flecha de orientación), la pelota,
y los arcos. Se usa desde viewer.py o desde env.render() en modo "human".
"""

from __future__ import annotations

import math
from typing import Any

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.axes import Axes

from .observation import FIELD_HALF_X, FIELD_HALF_Y


# Constantes visuales
ROBOT_RADIUS = 0.04
BALL_RADIUS = 0.025
GOAL_HALF_WIDTH = 0.20


def setup_field(ax: Axes) -> dict[str, Any]:
    """Dibuja el campo, arcos, línea de medio y devuelve handles para los
    objetos dinámicos (robot, pelota, flecha de orientación)."""
    ax.set_xlim(-FIELD_HALF_X - 0.05, FIELD_HALF_X + 0.05)
    ax.set_ylim(-FIELD_HALF_Y - 0.05, FIELD_HALF_Y + 0.05)
    ax.set_aspect("equal")
    ax.set_facecolor("#1f7a1f")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    # Borde del campo
    field = mpatches.Rectangle(
        (-FIELD_HALF_X, -FIELD_HALF_Y),
        2 * FIELD_HALF_X,
        2 * FIELD_HALF_Y,
        linewidth=2,
        edgecolor="white",
        facecolor="none",
    )
    ax.add_patch(field)

    # Línea de medio campo
    ax.plot([0, 0], [-FIELD_HALF_Y, FIELD_HALF_Y], "w--", linewidth=1, alpha=0.6)

    # Arcos (líneas blancas dentro del campo)
    ax.plot(
        [FIELD_HALF_X, FIELD_HALF_X],
        [-GOAL_HALF_WIDTH, GOAL_HALF_WIDTH],
        "w-",
        linewidth=4,
    )
    ax.plot(
        [-FIELD_HALF_X, -FIELD_HALF_X],
        [-GOAL_HALF_WIDTH, GOAL_HALF_WIDTH],
        "w-",
        linewidth=4,
    )

    # Etiquetas de arco
    ax.text(
        FIELD_HALF_X + 0.02, 0, "GOAL\n(rival)",
        color="yellow", fontsize=8, ha="left", va="center",
    )
    ax.text(
        -FIELD_HALF_X - 0.02, 0, "GOAL\n(propio)",
        color="cyan", fontsize=8, ha="right", va="center",
    )

    # Robot azul (círculo)
    robot_circle = mpatches.Circle(
        (0, 0), ROBOT_RADIUS, color="#3399ff", ec="white", linewidth=1.5, zorder=5
    )
    ax.add_patch(robot_circle)

    # Flecha de orientación del robot
    robot_arrow = ax.annotate(
        "",
        xy=(0, 0),
        xytext=(0, 0),
        arrowprops=dict(arrowstyle="->", color="white", lw=2),
        zorder=6,
    )

    # Pelota (naranja)
    ball_circle = mpatches.Circle(
        (0, 0), BALL_RADIUS, color="#ff8800", ec="white", linewidth=1, zorder=4
    )
    ax.add_patch(ball_circle)

    # Texto de info (tick, reward acumulado)
    info_text = ax.text(
        -FIELD_HALF_X, FIELD_HALF_Y + 0.02, "",
        color="white", fontsize=9, ha="left", va="bottom",
        bbox=dict(facecolor="black", alpha=0.5, pad=2, edgecolor="none"),
    )

    return {
        "robot_circle": robot_circle,
        "robot_arrow": robot_arrow,
        "ball_circle": ball_circle,
        "info_text": info_text,
    }


def update_field(
    handles: dict[str, Any],
    robot_pos: tuple[float, float],
    robot_theta: float,
    ball_pos: tuple[float, float],
    info: str,
) -> None:
    """Actualiza la posición de robot, flecha, pelota y texto."""
    rx, ry = robot_pos
    bx, by = ball_pos
    handles["robot_circle"].center = (rx, ry)

    arrow_len = 0.08
    handles["robot_arrow"].xy = (
        rx + arrow_len * math.cos(robot_theta),
        ry + arrow_len * math.sin(robot_theta),
    )
    handles["robot_arrow"].set_position((rx, ry))

    handles["ball_circle"].center = (bx, by)
    handles["info_text"].set_text(info)
