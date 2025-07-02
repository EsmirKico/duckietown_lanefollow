#!/usr/bin/env python3
"""
Test runner for Duckietown Lane Following with Dynamic Obstacle Avoidance.

This script provides various options for running different types of tests.
"""

import argparse
import subprocess
import sys
import os
from pathlib import Path


def run_command(cmd, description=""):
    """Run a command and handle errors."""
    print(f"\n{'='*60}")
    print(f"Running: {description}")
    print(f"Command: {' '.join(cmd)}")
    print(f"{'='*60}")
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(result.stdout)
        if result.stderr:
            print("STDERR:", result.stderr)
        return True
    except subprocess.CalledProcessError as e:
        print(f"ERROR: {e}")
        print(f"STDOUT: {e.stdout}")
        print(f"STDERR: {e.stderr}")
        return False


def check_dependencies():
    """Check if required dependencies are installed."""
    print("Checking test dependencies...")
    
    required_packages = [
        'pytest', 'numpy', 'opencv-python', 'mock'
    ]
    
    missing_packages = []
    for package in required_packages:
        try:
            __import__(package.replace('-', '_'))
            print(f"✓ {package}")
        except ImportError:
            print(f"✗ {package} (missing)")
            missing_packages.append(package)
    
    if missing_packages:
        print(f"\nMissing packages: {', '.join(missing_packages)}")
        print("Install with: pip install " + " ".join(missing_packages))
        return False
    
    return True


def run_unit_tests(verbose=False, coverage=False):
    """Run unit tests."""
    cmd = ['python', '-m', 'pytest', 'tests/unit/']
    
    if verbose:
        cmd.append('-v')
    
    if coverage:
        cmd.extend(['--cov=packages/', '--cov-report=html', '--cov-report=term'])
    
    cmd.extend(['-m', 'unit'])
    
    return run_command(cmd, "Unit Tests")


def run_integration_tests(verbose=False):
    """Run integration tests."""
    cmd = ['python', '-m', 'pytest', 'tests/integration/']
    
    if verbose:
        cmd.append('-v')
    
    cmd.extend(['-m', 'integration'])
    
    return run_command(cmd, "Integration Tests")


def run_simulation_tests(verbose=False):
    """Run simulation tests."""
    cmd = ['python', '-m', 'pytest', 'tests/simulation/']
    
    if verbose:
        cmd.append('-v')
    
    cmd.extend(['-m', 'simulation'])
    
    return run_command(cmd, "Simulation Tests")


def run_safety_tests(verbose=False):
    """Run safety-critical tests."""
    cmd = ['python', '-m', 'pytest']
    
    if verbose:
        cmd.append('-v')
    
    cmd.extend(['-m', 'safety', '--tb=short'])
    
    return run_command(cmd, "Safety-Critical Tests")


def run_performance_tests(verbose=False):
    """Run performance tests."""
    cmd = ['python', '-m', 'pytest']
    
    if verbose:
        cmd.append('-v')
    
    cmd.extend(['-m', 'slow', '--durations=10'])
    
    return run_command(cmd, "Performance Tests")


def run_all_tests(verbose=False, coverage=False):
    """Run all tests."""
    cmd = ['python', '-m', 'pytest', 'tests/']
    
    if verbose:
        cmd.append('-v')
    
    if coverage:
        cmd.extend(['--cov=packages/', '--cov-report=html', '--cov-report=term'])
    
    return run_command(cmd, "All Tests")


def generate_test_report():
    """Generate a comprehensive test report."""
    cmd = [
        'python', '-m', 'pytest', 'tests/',
        '--html=test_report.html', '--self-contained-html',
        '--cov=packages/', '--cov-report=html',
        '--junit-xml=test_results.xml'
    ]
    
    return run_command(cmd, "Generating Test Report")


def lint_code():
    """Run code linting."""
    print("\n" + "="*60)
    print("Running Code Linting")
    print("="*60)
    
    # Check if flake8 is available
    try:
        import flake8
        cmd = ['python', '-m', 'flake8', 'packages/', 'tests/', '--max-line-length=100']
        return run_command(cmd, "Flake8 Linting")
    except ImportError:
        print("Flake8 not installed, skipping linting")
        return True


def format_code():
    """Format code with black."""
    print("\n" + "="*60)
    print("Formatting Code")
    print("="*60)
    
    try:
        import black
        cmd = ['python', '-m', 'black', 'packages/', 'tests/', '--line-length=100']
        return run_command(cmd, "Black Code Formatting")
    except ImportError:
        print("Black not installed, skipping formatting")
        return True


def main():
    """Main test runner function."""
    parser = argparse.ArgumentParser(
        description="Test runner for Duckietown Dynamic Obstacle Avoidance",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_tests.py --unit                    # Run only unit tests
  python run_tests.py --integration --verbose   # Run integration tests with verbose output
  python run_tests.py --all --coverage          # Run all tests with coverage
  python run_tests.py --safety                  # Run safety-critical tests only
  python run_tests.py --report                  # Generate comprehensive test report
        """
    )
    
    # Test type options
    parser.add_argument('--unit', action='store_true', help='Run unit tests')
    parser.add_argument('--integration', action='store_true', help='Run integration tests')
    parser.add_argument('--simulation', action='store_true', help='Run simulation tests')
    parser.add_argument('--safety', action='store_true', help='Run safety-critical tests')
    parser.add_argument('--performance', action='store_true', help='Run performance tests')
    parser.add_argument('--all', action='store_true', help='Run all tests')
    
    # Output options
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    parser.add_argument('--coverage', action='store_true', help='Generate coverage report')
    parser.add_argument('--report', action='store_true', help='Generate HTML test report')
    
    # Code quality options
    parser.add_argument('--lint', action='store_true', help='Run code linting')
    parser.add_argument('--format', action='store_true', help='Format code with black')
    
    # Utility options
    parser.add_argument('--check-deps', action='store_true', help='Check dependencies only')
    
    args = parser.parse_args()
    
    # If no specific test type is selected, run all tests
    if not any([args.unit, args.integration, args.simulation, args.safety, 
                args.performance, args.all, args.report, args.lint, 
                args.format, args.check_deps]):
        args.all = True
    
    # Check dependencies first
    if not check_dependencies():
        print("\n❌ Dependency check failed!")
        return 1
    
    if args.check_deps:
        print("\n✅ All dependencies are available!")
        return 0
    
    # Track test results
    results = []
    
    # Run code formatting if requested
    if args.format:
        results.append(("Code Formatting", format_code()))
    
    # Run linting if requested
    if args.lint:
        results.append(("Code Linting", lint_code()))
    
    # Run tests based on arguments
    if args.unit:
        results.append(("Unit Tests", run_unit_tests(args.verbose, args.coverage)))
    
    if args.integration:
        results.append(("Integration Tests", run_integration_tests(args.verbose)))
    
    if args.simulation:
        results.append(("Simulation Tests", run_simulation_tests(args.verbose)))
    
    if args.safety:
        results.append(("Safety Tests", run_safety_tests(args.verbose)))
    
    if args.performance:
        results.append(("Performance Tests", run_performance_tests(args.verbose)))
    
    if args.all:
        results.append(("All Tests", run_all_tests(args.verbose, args.coverage)))
    
    if args.report:
        results.append(("Test Report", generate_test_report()))
    
    # Print summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    all_passed = True
    for test_name, passed in results:
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{test_name:30} {status}")
        if not passed:
            all_passed = False
    
    if all_passed:
        print(f"\n🎉 All tests passed! ({len(results)} test suites)")
        return 0
    else:
        failed_count = sum(1 for _, passed in results if not passed)
        print(f"\n💥 {failed_count} test suite(s) failed!")
        return 1


if __name__ == '__main__':
    sys.exit(main()) 