#!/usr/bin/env python
"""Comprehensive test runner for MuJoCo MCP project."""

import sys
import os
import pytest
import coverage
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root / "src"))


def run_tests():
    """Run all tests with coverage reporting."""
    
    print("=" * 80)
    print("MuJoCo MCP Test Suite")
    print("=" * 80)
    
    # Initialize coverage
    cov = coverage.Coverage(source=["mujoco_mcp"])
    cov.start()
    
    # Test categories
    test_suites = {
        "Unit Tests": [
            "tests/test_simulation_comprehensive.py",
            "tests/test_server_comprehensive.py", 
            "tests/test_auth_manager_comprehensive.py",
        ],
        "Integration Tests": [
            "tests/test_simulation_comprehensive.py::TestSimulationIntegration",
            "tests/test_server_comprehensive.py::TestAuthorizationIntegration",
        ],
        "Security Tests": [
            "-m", "security"
        ],
        "Performance Tests": [
            "-m", "benchmark"
        ]
    }
    
    all_passed = True
    results = {}
    
    # Run each test suite
    for suite_name, test_args in test_suites.items():
        print(f"\n{'=' * 40}")
        print(f"Running {suite_name}")
        print(f"{'=' * 40}")
        
        if isinstance(test_args, list) and test_args[0] == "-m":
            # Marker-based tests
            args = ["-v"] + test_args
        else:
            # File-based tests
            args = ["-v"] + test_args
        
        # Add common arguments
        args.extend([
            "--tb=short",
            "--maxfail=5",
            "-W", "ignore::DeprecationWarning"
        ])
        
        # Run tests
        exit_code = pytest.main(args)
        results[suite_name] = exit_code == 0
        all_passed = all_passed and (exit_code == 0)
    
    # Stop coverage and generate report
    cov.stop()
    cov.save()
    
    print("\n" + "=" * 80)
    print("Coverage Report")
    print("=" * 80)
    cov.report()
    
    # Generate HTML coverage report
    html_dir = project_root / "htmlcov"
    cov.html_report(directory=str(html_dir))
    print(f"\nDetailed HTML coverage report generated in: {html_dir}")
    
    # Summary
    print("\n" + "=" * 80)
    print("Test Summary")
    print("=" * 80)
    
    for suite_name, passed in results.items():
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{suite_name}: {status}")
    
    # Overall result
    print("\n" + "=" * 80)
    if all_passed:
        print("✅ All tests passed!")
    else:
        print("❌ Some tests failed!")
    print("=" * 80)
    
    return 0 if all_passed else 1


def run_specific_test(test_path):
    """Run a specific test file or test case."""
    print(f"Running specific test: {test_path}")
    args = ["-v", "-s", test_path]
    return pytest.main(args)


def run_quick_tests():
    """Run quick smoke tests."""
    print("Running quick smoke tests...")
    args = [
        "-v",
        "-k", "not slow and not benchmark",
        "--maxfail=1",
        "tests/"
    ]
    return pytest.main(args)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "--quick":
            exit_code = run_quick_tests()
        elif sys.argv[1] == "--specific" and len(sys.argv) > 2:
            exit_code = run_specific_test(sys.argv[2])
        else:
            print("Usage:")
            print("  python run_tests.py          # Run all tests")
            print("  python run_tests.py --quick  # Run quick tests")
            print("  python run_tests.py --specific <test_path>  # Run specific test")
            exit_code = 1
    else:
        exit_code = run_tests()
    
    sys.exit(exit_code)