# Scripts Directory

Utility and development scripts for MuJoCo MCP.

## Development Scripts

### Setup and Configuration
- **`setup-dev.sh`** - Set up development environment
- **`format.sh`** - Format code with Black, isort, and Ruff
- **`quality-check.sh`** - Run full code quality checks (linting, type checking, security)

### Viewer Launcher
- **`run_py_viewer.sh`** - Launch WebRTC browser viewer
  ```bash
  ./scripts/run_py_viewer.sh
  # Open http://localhost:8000 in your browser
  ```

## Testing Scripts

### Quick Tests
- **`quick_internal_test.py`** - Fast internal functionality check
  ```bash
  python scripts/quick_internal_test.py
  ```

### Browser Tests
- **`run_browser_tests.py`** - Browser UI and integration tests
  ```bash
  python scripts/run_browser_tests.py
  ```

- **`run_all_browser_tests.py`** - Comprehensive browser test suite
  ```bash
  python scripts/run_all_browser_tests.py
  ```

### Complete Test Suites
- **`run_all_tests.sh`** - Full pre-release test suite
  ```bash
  ./scripts/run_all_tests.sh
  ```

- **`test_local_install.sh`** - Test local package installation
  ```bash
  ./scripts/test_local_install.sh
  ```

## Utility Scripts

### LLM Scene Generation
- **`text_llm.py`** - Generate MuJoCo scenes from natural language
  ```bash
  export OPENAI_API_KEY="sk-..."
  python scripts/text_llm.py "create a cart pole with a 2m long pole"
  ```

### Web Server
- **`web_server.py`** - Standalone web server for MCP client
  ```bash
  python scripts/web_server.py
  # Open http://localhost:8080
  ```

### Monitoring
- **`estimate_client_capacity.py`** - Estimate multi-client capacity
  ```bash
  python scripts/estimate_client_capacity.py
  ```

## Usage Patterns

### Before Committing
```bash
# Format code
./scripts/format.sh

# Check quality
./scripts/quality-check.sh
```

### Before Release
```bash
# Run all tests
./scripts/run_all_tests.sh

# Test local installation
./scripts/test_local_install.sh
```

### Development Workflow
```bash
# Start viewer for development
./scripts/run_py_viewer.sh

# In another terminal, generate scenes
python scripts/text_llm.py "your scene description"

# Run quick tests
python scripts/quick_internal_test.py
```

## Notes

- All scripts should be run from the project root directory
- Most bash scripts are designed to work with both system Python and virtual environments
- Python scripts automatically adjust `PYTHONPATH` to include `src/` directory
