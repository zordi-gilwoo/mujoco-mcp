# Contributing to MuJoCo MCP

We love your input! We want to make contributing to MuJoCo MCP as easy and transparent as possible, whether it's:

- Reporting a bug
- Discussing the current state of the code
- Submitting a fix
- Proposing new features
- Becoming a maintainer

## We Develop with Github

We use GitHub to host code, to track issues and feature requests, as well as accept pull requests.

## We Use [Github Flow](https://guides.github.com/introduction/flow/index.html)

Pull requests are the best way to propose changes to the codebase:

1. Fork the repo and create your branch from `main`.
2. If you've added code that should be tested, add tests.
3. If you've changed APIs, update the documentation.
4. Ensure the test suite passes.
5. Make sure your code follows the style guidelines.
6. Issue that pull request!

## Any contributions you make will be under the MIT Software License

In short, when you submit code changes, your submissions are understood to be under the same [MIT License](LICENSE) that covers the project. Feel free to contact the maintainers if that's a concern.

## Report bugs using Github's [issues](https://github.com/yourusername/mujoco-mcp/issues)

We use GitHub issues to track public bugs. Report a bug by [opening a new issue](https://github.com/yourusername/mujoco-mcp/issues/new); it's that easy!

## Write bug reports with detail, background, and sample code

**Great Bug Reports** tend to have:

- A quick summary and/or background
- Steps to reproduce
  - Be specific!
  - Give sample code if you can
- What you expected would happen
- What actually happens
- Notes (possibly including why you think this might be happening, or stuff you tried that didn't work)

## Development Process

### Setting Up Your Development Environment

```bash
# Clone your fork
git clone https://github.com/yourusername/mujoco-mcp.git
cd mujoco-mcp

# Create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install in development mode with test dependencies
pip install -e ".[test]"
```

### Running Tests

**Always write tests for new features!**

```bash
# Run all tests
python tests/run_tests.py

# Run specific test file
pytest tests/test_server_comprehensive.py -v

# Run with coverage
pytest --cov=mujoco_mcp tests/
```

### Code Style

We use [Black](https://github.com/psf/black) for Python code formatting:

```bash
# Format code
black src/ tests/

# Check formatting
black --check src/ tests/
```

We also use [ruff](https://github.com/charliermarsh/ruff) for linting:

```bash
# Run linter
ruff src/ tests/

# Fix auto-fixable issues
ruff --fix src/ tests/
```

### Documentation

- Use Google-style docstrings for all public functions and classes
- Update README.md if you change functionality
- Add inline comments for complex logic

### Security Considerations

**IMPORTANT**: Since MuJoCo MCP controls physical simulations:

1. **Never** allow arbitrary code execution
2. **Always** validate input parameters
3. **Implement** appropriate rate limiting
4. **Test** force/velocity limits thoroughly
5. **Document** any security implications of new features

### Testing Guidelines

1. **Write tests first** (TDD approach)
2. **Aim for >85% code coverage**
3. **Test edge cases** and error conditions
4. **Include integration tests** for new features
5. **Add performance benchmarks** for critical paths

Example test structure:

```python
def test_feature_normal_case():
    """Test feature under normal conditions."""
    # Arrange
    sim = MockSimulation()
    
    # Act
    result = feature_under_test(sim, valid_input)
    
    # Assert
    assert result == expected_output

def test_feature_edge_case():
    """Test feature with edge case inputs."""
    # Test boundary conditions, None values, etc.

def test_feature_error_handling():
    """Test feature error handling."""
    with pytest.raises(ValueError):
        feature_under_test(invalid_input)
```

### Commit Messages

Use clear and descriptive commit messages:

- `feat: Add trajectory planning tool`
- `fix: Correct joint limit validation`
- `docs: Update installation instructions`
- `test: Add tests for grasp controller`
- `refactor: Simplify authorization flow`
- `perf: Optimize simulation step performance`

### Pull Request Process

1. Update the README.md with details of changes if needed
2. Update the CHANGELOG.md with your changes
3. Ensure all tests pass and coverage is maintained
4. Request review from maintainers
5. Address review feedback promptly

## Code of Conduct

### Our Pledge

We pledge to make participation in our project a harassment-free experience for everyone, regardless of age, body size, disability, ethnicity, gender identity and expression, level of experience, nationality, personal appearance, race, religion, or sexual identity and orientation.

### Our Standards

Examples of behavior that contributes to creating a positive environment include:

- Using welcoming and inclusive language
- Being respectful of differing viewpoints and experiences
- Gracefully accepting constructive criticism
- Focusing on what is best for the community
- Showing empathy towards other community members

### Attribution

This Code of Conduct is adapted from the [Contributor Covenant][homepage], version 1.4.

[homepage]: http://contributor-covenant.org/version/1/4/

## Questions?

Feel free to open an issue with the tag "question" or reach out in our [discussions](https://github.com/yourusername/mujoco-mcp/discussions).

Thank you for contributing to MuJoCo MCP! 🚀