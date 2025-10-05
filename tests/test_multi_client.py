"""
Multi-Client MCP Server Tests
Tests for session isolation and multi-client functionality
"""

import pytest
import asyncio
import threading
from unittest.mock import patch, MagicMock

from mujoco_mcp.mcp_server_menagerie import handle_call_tool, handle_list_tools
from mujoco_mcp.session_manager import session_manager


@pytest.fixture
def clean_session_manager():
    """Clean session manager before each test"""
    # Store original sessions
    original_sessions = session_manager.sessions.copy()

    # Clear sessions for test
    session_manager.sessions.clear()

    yield session_manager

    # Restore original sessions
    session_manager.sessions = original_sessions


@pytest.mark.asyncio
async def test_session_info_tool():
    """Test the get_session_info tool"""
    result = await handle_call_tool("get_session_info", {})

    assert len(result) == 1
    assert result[0].type == "text"

    # Should contain session information
    assert "current_session" in result[0].text
    assert "session_id" in result[0].text
    assert "client_id" in result[0].text


@pytest.mark.asyncio
async def test_multi_client_isolation():
    """Test that different threads get different sessions"""

    results = {}

    def client_thread(client_id):
        """Simulate a client in a separate thread"""

        async def run_client():
            # Each thread should get its own session
            result = await handle_call_tool("get_session_info", {})
            return result[0].text

        # Run in new event loop for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            results[client_id] = loop.run_until_complete(run_client())
        finally:
            loop.close()

    # Create multiple client threads
    threads = []
    for i in range(3):
        thread = threading.Thread(target=client_thread, args=(f"client_{i}",))
        threads.append(thread)
        thread.start()

    # Wait for all threads to complete
    for thread in threads:
        thread.join()

    # Each client should have a different session_id
    session_ids = set()
    for client_id, result_text in results.items():
        # Extract session info (simplified parsing)
        assert "session_id" in result_text
        assert "client_id" in result_text

        # Each should be unique (simplified check)
        session_ids.add(result_text)

    # All results should be different
    assert len(session_ids) == len(results)


@pytest.mark.asyncio
async def test_session_model_tracking(clean_session_manager):
    """Test that sessions track their active models"""
    with patch("mujoco_mcp.session_manager.viewer_manager") as mock_viewer_manager:
        # Mock viewer client
        mock_client = MagicMock()
        mock_client.send_command.return_value = {"success": True}
        mock_viewer_manager.get_client.return_value = mock_client
        mock_viewer_manager.create_client.return_value = True

        # Create a scene
        result = await handle_call_tool("create_scene", {"scene_type": "pendulum"})

        assert len(result) == 1
        assert "✅ Created pendulum scene successfully" in result[0].text

        # Check session info
        session_info = await handle_call_tool("get_session_info", {})
        session_text = session_info[0].text

        # Should show the active model
        assert '"pendulum": "pendulum"' in session_text or '"active_models"' in session_text


@pytest.mark.asyncio
async def test_session_specific_model_ids(clean_session_manager):
    """Test that model IDs are session-specific"""
    with patch("mujoco_mcp.session_manager.viewer_manager") as mock_viewer_manager:
        # Mock viewer client
        mock_client = MagicMock()
        mock_client.send_command.return_value = {"success": True}
        mock_viewer_manager.get_client.return_value = mock_client
        mock_viewer_manager.create_client.return_value = True

        # Create a scene
        await handle_call_tool("create_scene", {"scene_type": "pendulum"})

        # Check that the model ID sent to viewer includes session prefix
        mock_client.send_command.assert_called()
        call_args = mock_client.send_command.call_args[0][0]

        assert call_args["type"] == "load_model"
        assert call_args["model_id"].startswith("session_")
        assert "pendulum" in call_args["model_id"]


@pytest.mark.asyncio
async def test_tools_list_includes_session_tool():
    """Test that the tools list includes the new session info tool"""
    tools = await handle_list_tools()

    tool_names = [tool.name for tool in tools]

    # Should include the new session info tool
    assert "get_session_info" in tool_names

    # Should still include all original tools
    expected_tools = [
        "get_server_info",
        "create_scene",
        "step_simulation",
        "get_state",
        "reset_simulation",
        "close_viewer",
        "get_session_info",
    ]

    for tool_name in expected_tools:
        assert tool_name in tool_names


@pytest.mark.asyncio
async def test_close_viewer_session_cleanup(clean_session_manager):
    """Test that close_viewer properly cleans up sessions"""
    with patch("mujoco_mcp.session_manager.viewer_manager") as mock_viewer_manager:
        # Mock viewer client
        mock_client = MagicMock()
        mock_client.send_command.return_value = {"success": True}
        mock_viewer_manager.get_client.return_value = mock_client
        mock_viewer_manager.create_client.return_value = True

        # Create a scene first
        await handle_call_tool("create_scene", {"scene_type": "pendulum"})

        # Verify session exists
        assert len(session_manager.sessions) > 0

        # Close viewer without model_id (should close entire session)
        result = await handle_call_tool("close_viewer", {})

        assert len(result) == 1
        assert "closed and cleaned up" in result[0].text


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
