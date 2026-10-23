"""Tests for physical wand button edge handling."""

from cf_control.wand_button_node import ButtonEdges


def test_button_emits_only_once_per_press():
    edges = ButtonEdges()

    assert edges.pressed('stop', 0) is False
    assert edges.pressed('stop', 1) is True
    assert edges.pressed('stop', 1) is False
    assert edges.pressed('stop', 0) is False
    assert edges.pressed('stop', 1) is True
