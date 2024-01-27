"""Defines a dummy test."""

import pytest


def test_dummy() -> None:
    assert True


@pytest.mark.slow
def test_slow() -> None:
    assert True


@pytest.mark.has_gpu
def test_gpu() -> None:
    assert True


@pytest.mark.has_mps
def test_mps() -> None:
    assert True
