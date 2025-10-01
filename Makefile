.PHONY: help install test run build docker clean

# Default target
help:
	@echo "Available commands:"
	@echo "  install     Install dependencies"
	@echo "  test        Run tests"
	@echo "  run         Run the development server"
	@echo "  build       Build Docker image"
	@echo "  docker      Run with Docker Compose"
	@echo "  clean       Clean build artifacts"
	@echo "  lint        Run code formatting and linting"
	@echo "  dev-check   Run development validation"

# Install dependencies
install:
	pip install -r requirements.txt
	pip install pytest pytest-asyncio httpx pytest-mock

# Run tests
test:
	python -m pytest test_py_remote_viewer_comprehensive.py -v
	python -m py_remote_viewer.dev_check

# Run development server
run:
	python -m py_remote_viewer --debug --port 8000

# Run with specific port
run-port:
	python -m py_remote_viewer --port $(PORT)

# Development validation
dev-check:
	python -m py_remote_viewer.dev_check

# Build Docker image
build:
	docker build -t mujoco-viewer:latest .

# Run with Docker Compose
docker:
	docker-compose up --build

# Run Docker container
docker-run:
	docker run -p 8000:8000 mujoco-viewer:latest

# Clean build artifacts
clean:
	find . -type f -name "*.pyc" -delete
	find . -type d -name "__pycache__" -delete
	find . -type d -name "*.egg-info" -exec rm -rf {} +
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	rm -rf build/ dist/ logs/

# Code formatting and linting
lint:
	python -m black py_remote_viewer/ test_*.py --line-length 88
	python -m flake8 py_remote_viewer/ test_*.py --max-line-length 88

# Quick server startup
quick:
	./scripts/run_py_viewer.sh

# Health check
health:
	curl -f http://localhost:8000/api/health || echo "Server not running"

# API stats
stats:
	curl -s http://localhost:8000/api/stats | python -m json.tool

# Full deployment test
deploy-test: build
	docker run -d --name mujoco-test -p 8001:8000 mujoco-viewer:latest
	sleep 10
	curl -f http://localhost:8001/api/health
	docker stop mujoco-test
	docker rm mujoco-test

# Kubernetes deployment
k8s-deploy:
	kubectl apply -f k8s-deployment.yaml

# Kubernetes cleanup
k8s-clean:
	kubectl delete -f k8s-deployment.yaml