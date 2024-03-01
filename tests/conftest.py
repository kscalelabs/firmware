"""Defines PyTest configuration for the project."""

from _pytest.python import Function


def pytest_collection_modifyitems(items: list[Function]) -> None:
    items.sort(key=lambda x: x.get_closest_marker("slow") is not None)
