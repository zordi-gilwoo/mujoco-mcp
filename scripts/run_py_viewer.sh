#!/bin/bash
# MuJoCo Remote Viewer Launcher Script

set -e

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "🚀 Starting MuJoCo Remote Viewer"
echo "Project root: $PROJECT_ROOT"

# Check if virtual environment should be used
if [[ -n "$VIRTUAL_ENV" ]]; then
    echo "✅ Using virtual environment: $VIRTUAL_ENV"
elif [[ -f "$PROJECT_ROOT/venv/bin/activate" ]]; then
    echo "🔧 Activating virtual environment..."
    source "$PROJECT_ROOT/venv/bin/activate"
elif [[ -f "$PROJECT_ROOT/.venv/bin/activate" ]]; then
    echo "🔧 Activating virtual environment..."
    source "$PROJECT_ROOT/.venv/bin/activate"
else
    echo "⚠️  No virtual environment detected. Using system Python."
fi

# Set default environment variables
export VIEWER_HOST="${VIEWER_HOST:-localhost}"
export VIEWER_PORT="${VIEWER_PORT:-8000}"
export LOG_LEVEL="${LOG_LEVEL:-DEBUG}"
export DEBUG_MODE="${DEBUG_MODE:-1}"

# Auto-detect headless environment and set OpenGL backend
# Check if DISPLAY is not set or if we're on a headless server
if [[ -z "$MUJOCO_GL" ]]; then
    if [[ -z "$DISPLAY" ]] || ! command -v xdpyinfo &> /dev/null || ! xdpyinfo &> /dev/null; then
        echo "🖥️  Headless environment detected - defaulting to OSMesa rendering"
        export MUJOCO_GL=osmesa
        export PYOPENGL_PLATFORM=osmesa
    fi
elif [[ "$MUJOCO_GL" == "osmesa" ]]; then
    export PYOPENGL_PLATFORM=osmesa
    echo "🖥️  Using OSMesa for headless rendering"
fi

echo "📋 Configuration:"
echo "  Host: $VIEWER_HOST"
echo "  Port: $VIEWER_PORT"
echo "  Log level: $LOG_LEVEL"
echo "  Debug mode: $DEBUG_MODE"

# Change to project directory
cd "$PROJECT_ROOT"

# Check if dependencies are installed
echo "🔍 Checking dependencies..."
if ! python -c "import fastapi, aiortc, uvicorn" 2>/dev/null; then
    echo "❌ Missing dependencies. Installing from requirements.txt..."
    pip install -r requirements.txt
else
    echo "✅ Dependencies look good"
fi

# Run the server
echo "🌐 Starting server at http://$VIEWER_HOST:$VIEWER_PORT"
echo "📱 Open your web browser and navigate to the URL above"
echo "🛑 Press Ctrl+C to stop the server"
echo ""

# Start the server using the Python module
python -m py_remote_viewer.server

echo "👋 Server stopped"
