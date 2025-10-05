"""Shared pytest configuration for the project test suite."""


def pytest_addoption(parser):
    """Register legacy config options expected by upstream plugins."""

    # `pytest-asyncio` is an optional dependency in this workspace, but the existing
    # pytest.ini references the `asyncio_mode` setting. Register the option here so
    # pytest does not error when the plugin is absent.
    parser.addini(
        "asyncio_mode", "Default asyncio mode for optional pytest-asyncio", default="auto"
    )
