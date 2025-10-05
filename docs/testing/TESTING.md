# Testing Guide

## Quick Start

### Run All Tests
```bash
# Run complete test suite
pytest tests/ -v

# Run with coverage
pytest tests/ --cov=src --cov-report=html
```

---

## Test Categories

### 1. MCP Server Tests
```bash
# Core MCP functionality
pytest tests/mcp/ -v

# Integration tests
pytest tests/integration/ -v
```

### 2. WebRTC Viewer Tests
```bash
# Unit tests (16 tests)
pytest tests/py_remote_viewer/test_py_remote_viewer_comprehensive.py -v

# End-to-end browser tests
pytest tests/py_remote_viewer/test_py_remote_viewer_e2e.py -v

# Development validation
python -m py_remote_viewer.dev_check
```

### 3. RL Integration Tests
```bash
# Basic RL tests
pytest tests/rl/ -v
```

### 4. Browser/UI Tests
```bash
# Playwright-based UI testing
pytest tests/test_e2e_browser.py -v
pytest tests/test_mcp_browser_integration.py -v
```

### 5. Performance Tests
```bash
# Benchmarks
pytest tests/performance/ -v
```

---

## Setup

### Install Test Dependencies
```bash
pip install pytest pytest-cov pytest-asyncio playwright
```

### Install Browsers (for UI tests)
```bash
playwright install chromium
```

---

## Development Checks

### Quick Validation
```bash
# MCP server check
python -m mujoco_mcp --check

# WebRTC viewer check
python -m py_remote_viewer.dev_check
```

### Test Scripts
```bash
# Basic internal tests
python scripts/quick_internal_test.py

# Browser tests
python run_browser_tests.py
```

---

## Configuration

### Environment Variables
```bash
export TEST_HEADLESS=true      # Run headless
export TEST_TIMEOUT=30          # Test timeout
export MCP_TEST_PORT=8888       # MCP port
export WEBRTC_TEST_PORT=8000    # WebRTC port
```

### Test Files
- `pytest.ini` - Pytest settings
- `tests/conftest_v0_8.py` - Test configuration

---

## Debugging

### Verbose Output
```bash
# Detailed output
pytest tests/ -v -s

# Show variables on failure
pytest tests/ --tb=long
```

### Debug Logging
```bash
export LOG_LEVEL=DEBUG
pytest tests/ -v
```

---

## CI/CD

Tests run automatically on:
- Pull requests
- Push to main branch
- Nightly builds

See `.github/workflows/` for configuration.

---

That's it! For detailed test documentation, see the code in `tests/`.
