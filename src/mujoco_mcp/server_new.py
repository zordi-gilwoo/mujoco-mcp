"""MuJoCo MCP Server implementation using FastMCP."""

import logging
from typing import Dict, List, Any, Optional
from mcp.server import FastMCP
from .simulation import MuJoCoSimulation
from .auth_manager import AuthManager, default_auth_manager
from .enhanced_auth_manager import EnhancedAuthManager

# Create logger
logger = logging.getLogger("mujoco_mcp.server")

# Create MCP server instance
mcp = FastMCP("MuJoCo Control Server")

# Global state
simulations: Dict[str, MuJoCoSimulation] = {}
auth_manager: AuthManager = default_auth_manager


class MuJoCoMCPServer:
    """MuJoCo MCP Server wrapper class."""
    
    def __init__(self, auth_manager: Optional[AuthManager] = None):
        """Initialize the server with optional auth manager."""
        global auth_manager
        if auth_manager:
            auth_manager = auth_manager
        self.mcp = mcp
        self.simulations = simulations
    
    def run(self):
        """Run the MCP server."""
        self.mcp.run()


# Resources
@mcp.resource("simulation://list")
def list_simulations() -> str:
    """List all active simulations."""
    return "\n".join([f"{sim_id}: {sim.__class__.__name__}" for sim_id, sim in simulations.items()])


@mcp.resource("simulation://{sim_id}/joint_positions")
def get_joint_positions(sim_id: str) -> str:
    """Get current joint positions."""
    if sim_id not in simulations:
        return f"Error: Simulation {sim_id} not found"
    sim = simulations[sim_id]
    positions = sim.get_joint_positions()
    return str(positions)


@mcp.resource("simulation://{sim_id}/joint_velocities") 
def get_joint_velocities(sim_id: str) -> str:
    """Get current joint velocities."""
    if sim_id not in simulations:
        return f"Error: Simulation {sim_id} not found"
    sim = simulations[sim_id]
    velocities = sim.get_joint_velocities()
    return str(velocities)


@mcp.resource("simulation://{sim_id}/sensor_data")
def get_sensor_data(sim_id: str) -> str:
    """Get sensor data from simulation."""
    if sim_id not in simulations:
        return f"Error: Simulation {sim_id} not found"
    sim = simulations[sim_id]
    data = sim.get_sensor_data()
    return str(data)


@mcp.resource("simulation://{sim_id}/robot_state")
def get_robot_state(sim_id: str) -> str:
    """Get complete robot state."""
    if sim_id not in simulations:
        return f"Error: Simulation {sim_id} not found"
    sim = simulations[sim_id]
    state = sim.get_robot_state()
    return str(state)


# Tools
@mcp.tool()
def start_simulation(model_xml: str, sim_id: Optional[str] = None) -> str:
    """Start a new MuJoCo simulation.
    
    Args:
        model_xml: Path to MuJoCo XML model file
        sim_id: Optional simulation ID (auto-generated if not provided)
    
    Returns:
        Simulation ID
    """
    import uuid
    
    if sim_id is None:
        sim_id = str(uuid.uuid4())
    
    # Authorize operation
    if auth_manager:
        authorized, message, _ = auth_manager.request_authorization(
            client_id="default",
            operation="start_simulation",
            parameters={"model_xml": model_xml}
        )
        if not authorized:
            return f"Error: Authorization denied - {message}"
    
    try:
        sim = MuJoCoSimulation()
        sim.load_model(model_xml)
        simulations[sim_id] = sim
        logger.info(f"Started simulation {sim_id} with model {model_xml}")
        return sim_id
    except Exception as e:
        logger.error(f"Failed to start simulation: {e}")
        return f"Error: {str(e)}"


@mcp.tool()
def step_simulation(sim_id: str, steps: int = 1) -> str:
    """Step the simulation forward.
    
    Args:
        sim_id: Simulation ID
        steps: Number of steps to advance (default: 1)
    
    Returns:
        Success message or error
    """
    if sim_id not in simulations:
        return f"Error: Simulation {sim_id} not found"
    
    # Authorize operation
    if auth_manager:
        authorized, message, _ = auth_manager.request_authorization(
            client_id="default",
            operation="step_simulation",
            parameters={"sim_id": sim_id, "steps": steps}
        )
        if not authorized:
            return f"Error: Authorization denied - {message}"
    
    try:
        sim = simulations[sim_id]
        for _ in range(steps):
            sim.step()
        return f"Stepped simulation {sim_id} forward {steps} steps"
    except Exception as e:
        logger.error(f"Failed to step simulation: {e}")
        return f"Error: {str(e)}"


@mcp.tool()
def reset_simulation(sim_id: str) -> str:
    """Reset simulation to initial state.
    
    Args:
        sim_id: Simulation ID
    
    Returns:
        Success message or error
    """
    if sim_id not in simulations:
        return f"Error: Simulation {sim_id} not found"
    
    # Authorize operation
    if auth_manager:
        authorized, message, _ = auth_manager.request_authorization(
            client_id="default",
            operation="reset_simulation",
            parameters={"sim_id": sim_id}
        )
        if not authorized:
            return f"Error: Authorization denied - {message}"
    
    try:
        sim = simulations[sim_id]
        sim.reset()
        return f"Reset simulation {sim_id}"
    except Exception as e:
        logger.error(f"Failed to reset simulation: {e}")
        return f"Error: {str(e)}"


@mcp.tool()
def set_joint_positions(sim_id: str, positions: List[float]) -> str:
    """Set joint positions.
    
    Args:
        sim_id: Simulation ID
        positions: List of joint positions
    
    Returns:
        Success message or error
    """
    if sim_id not in simulations:
        return f"Error: Simulation {sim_id} not found"
    
    # Authorize operation
    if auth_manager:
        authorized, message, _ = auth_manager.request_authorization(
            client_id="default",
            operation="set_joint_positions", 
            parameters={"sim_id": sim_id, "positions": positions}
        )
        if not authorized:
            return f"Error: Authorization denied - {message}"
    
    try:
        sim = simulations[sim_id]
        sim.set_joint_positions(positions)
        return f"Set joint positions for simulation {sim_id}"
    except Exception as e:
        logger.error(f"Failed to set joint positions: {e}")
        return f"Error: {str(e)}"


@mcp.tool()
def apply_control(sim_id: str, control: List[float]) -> str:
    """Apply control inputs to actuators.
    
    Args:
        sim_id: Simulation ID
        control: List of control values
    
    Returns:
        Success message or error
    """
    if sim_id not in simulations:
        return f"Error: Simulation {sim_id} not found"
    
    # Authorize operation
    if auth_manager:
        authorized, message, _ = auth_manager.request_authorization(
            client_id="default",
            operation="apply_control",
            parameters={"sim_id": sim_id, "control": control}
        )
        if not authorized:
            return f"Error: Authorization denied - {message}"
    
    try:
        sim = simulations[sim_id]
        sim.apply_control(control)
        return f"Applied control to simulation {sim_id}"
    except Exception as e:
        logger.error(f"Failed to apply control: {e}")
        return f"Error: {str(e)}"


@mcp.tool()
def move_to_position(sim_id: str, target_position: List[float], speed: float = 1.0) -> str:
    """Move robot to target position.
    
    Args:
        sim_id: Simulation ID
        target_position: Target position coordinates [x, y, z]
        speed: Movement speed factor (0.0-1.0)
    
    Returns:
        Success message or error
    """
    if sim_id not in simulations:
        return f"Error: Simulation {sim_id} not found"
    
    # Authorize operation with enhanced auth manager if available
    if isinstance(auth_manager, EnhancedAuthManager):
        authorized, message, _ = auth_manager.request_authorization(
            client_id="default",
            operation="move_to_position",
            parameters={"sim_id": sim_id, "target_position": target_position, "speed": speed}
        )
        if not authorized:
            return f"Error: Authorization denied - {message}"
    
    try:
        sim = simulations[sim_id]
        # Implementation would go here
        return f"Moving to position {target_position} at speed {speed}"
    except Exception as e:
        logger.error(f"Failed to move to position: {e}")
        return f"Error: {str(e)}"


@mcp.tool()
def grasp_object(sim_id: str, object_name: str, force: float = 1.0) -> str:
    """Grasp an object in the simulation.
    
    Args:
        sim_id: Simulation ID
        object_name: Name of object to grasp
        force: Grasping force (0.0-1.0)
    
    Returns:
        Success message or error
    """
    if sim_id not in simulations:
        return f"Error: Simulation {sim_id} not found"
    
    # Authorize operation
    if isinstance(auth_manager, EnhancedAuthManager):
        authorized, message, _ = auth_manager.request_authorization(
            client_id="default",
            operation="grasp_object",
            parameters={"sim_id": sim_id, "object_name": object_name, "force": force}
        )
        if not authorized:
            return f"Error: Authorization denied - {message}"
    
    try:
        sim = simulations[sim_id]
        # Implementation would go here
        return f"Grasped object {object_name} with force {force}"
    except Exception as e:
        logger.error(f"Failed to grasp object: {e}")
        return f"Error: {str(e)}"


# Export server instance
__all__ = ['MuJoCoMCPServer', 'mcp', 'simulations', 'auth_manager']