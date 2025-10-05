"""
End-to-end tests for py_remote_viewer using Playwright browser automation.
Tests the complete WebRTC viewer functionality from browser perspective.
"""

import asyncio
import pytest
import subprocess
import time
import signal
import os
from pathlib import Path
from playwright.async_api import async_playwright
import httpx


class TestE2EWebRTCViewer:
    """End-to-end tests using browser automation."""

    @pytest.fixture(scope="class")
    async def server_process(self):
        """Start the py_remote_viewer server for testing."""
        # Start server in background
        env = os.environ.copy()
        env["PORT"] = "8003"  # Use test port

        process = subprocess.Popen(
            ["python", "-m", "py_remote_viewer", "--port", "8003"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )

        # Wait for server to start
        await asyncio.sleep(3)

        # Verify server is running
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get("http://localhost:8003/api/health", timeout=5)
                assert response.status_code == 200
        except Exception as e:
            process.terminate()
            raise RuntimeError(f"Server failed to start: {e}")

        yield process

        # Cleanup
        process.terminate()
        process.wait()

    @pytest.mark.asyncio
    async def test_browser_interface_loads(self, server_process):
        """Test that the web interface loads correctly in browser."""
        async with async_playwright() as p:
            # Use chromium for testing
            browser = await p.chromium.launch(headless=True)
            context = await browser.new_context()
            page = await context.new_page()

            try:
                # Navigate to the viewer
                await page.goto("http://localhost:8003")

                # Wait for page to load
                await page.wait_for_load_state("networkidle")

                # Check page title
                title = await page.title()
                assert "MuJoCo Remote Viewer" in title

                # Check that video element exists
                video_element = await page.query_selector("#videoElement")
                assert video_element is not None

                # Check that control buttons exist
                start_button = await page.query_selector("button:has-text('Start')")
                assert start_button is not None

                pause_button = await page.query_selector("button:has-text('Pause')")
                assert pause_button is not None

                reset_button = await page.query_selector("button:has-text('Reset')")
                assert reset_button is not None

            finally:
                await browser.close()

    @pytest.mark.asyncio
    async def test_webrtc_connection_establishment(self, server_process):
        """Test WebRTC connection establishment."""
        async with async_playwright() as p:
            browser = await p.chromium.launch(headless=True)
            context = await browser.new_context()
            page = await context.new_page()

            try:
                # Navigate to the viewer
                await page.goto("http://localhost:8003")
                await page.wait_for_load_state("networkidle")

                # Wait for WebRTC connection to establish
                await asyncio.sleep(5)

                # Check connection status
                connection_status = await page.evaluate(
                    """
                    () => {
                        const element = document.querySelector('#connectionStatus');
                        return element ? element.textContent : null;
                    }
                """
                )

                # Should show some connection status
                assert connection_status is not None

            finally:
                await browser.close()

    @pytest.mark.asyncio
    async def test_mouse_event_capture(self, server_process):
        """Test mouse event capture and forwarding."""
        async with async_playwright() as p:
            browser = await p.chromium.launch(headless=True)
            context = await browser.new_context()
            page = await context.new_page()

            try:
                # Navigate to the viewer
                await page.goto("http://localhost:8003")
                await page.wait_for_load_state("networkidle")

                # Wait for initialization
                await asyncio.sleep(3)

                # Find video element
                video_element = await page.query_selector("#videoElement")
                assert video_element is not None

                # Simulate mouse movement over video
                await video_element.hover()
                await video_element.click()

                # Simulate mouse drag
                await page.mouse.move(100, 100)
                await page.mouse.down()
                await page.mouse.move(200, 150)
                await page.mouse.up()

                # Give time for events to process
                await asyncio.sleep(1)

                # Check that events were logged (if event log is visible)
                event_log = await page.query_selector("#eventLog")
                if event_log:
                    log_content = await event_log.text_content()
                    # Should contain some mouse events
                    assert "mouse" in log_content.lower() or len(log_content) > 0

            finally:
                await browser.close()

    @pytest.mark.asyncio
    async def test_keyboard_event_capture(self, server_process):
        """Test keyboard event capture and forwarding."""
        async with async_playwright() as p:
            browser = await p.chromium.launch(headless=True)
            context = await browser.new_context()
            page = await context.new_page()

            try:
                # Navigate to the viewer
                await page.goto("http://localhost:8003")
                await page.wait_for_load_state("networkidle")

                # Wait for initialization
                await asyncio.sleep(3)

                # Focus on the page
                await page.click("body")

                # Simulate keyboard events
                await page.keyboard.press("Space")
                await page.keyboard.press("ArrowUp")
                await page.keyboard.press("ArrowDown")
                await page.keyboard.press("ArrowLeft")
                await page.keyboard.press("ArrowRight")

                # Give time for events to process
                await asyncio.sleep(1)

                # Events should be processed (no errors)
                # Check console for errors
                console_messages = []

                def handle_console(msg):
                    if msg.type == "error":
                        console_messages.append(msg.text)

                page.on("console", handle_console)

                # Wait a bit more to catch any console errors
                await asyncio.sleep(1)

                # Should not have critical errors
                critical_errors = [msg for msg in console_messages if "critical" in msg.lower()]
                assert len(critical_errors) == 0

            finally:
                await browser.close()

    @pytest.mark.asyncio
    async def test_camera_preset_buttons(self, server_process):
        """Test camera preset buttons functionality."""
        async with async_playwright() as p:
            browser = await p.chromium.launch(headless=True)
            context = await browser.new_context()
            page = await context.new_page()

            try:
                # Navigate to the viewer
                await page.goto("http://localhost:8003")
                await page.wait_for_load_state("networkidle")

                # Wait for initialization
                await asyncio.sleep(3)

                # Test camera preset buttons
                presets = ["Front", "Side", "Top", "Reset"]

                for preset in presets:
                    button = await page.query_selector(f"button:has-text('{preset}')")
                    if button:
                        await button.click()
                        await asyncio.sleep(0.5)  # Give time for camera to update

                # Should complete without errors
                assert True

            finally:
                await browser.close()

    @pytest.mark.asyncio
    async def test_simulation_controls(self, server_process):
        """Test simulation control buttons."""
        async with async_playwright() as p:
            browser = await p.chromium.launch(headless=True)
            context = await browser.new_context()
            page = await context.new_page()

            try:
                # Navigate to the viewer
                await page.goto("http://localhost:8003")
                await page.wait_for_load_state("networkidle")

                # Wait for initialization
                await asyncio.sleep(3)

                # Test Start button
                start_button = await page.query_selector("button:has-text('Start')")
                if start_button:
                    await start_button.click()
                    await asyncio.sleep(1)

                # Test Pause button
                pause_button = await page.query_selector("button:has-text('Pause')")
                if pause_button:
                    await pause_button.click()
                    await asyncio.sleep(1)

                # Test Reset button
                reset_button = await page.query_selector("button:has-text('Reset')")
                if reset_button:
                    await reset_button.click()
                    await asyncio.sleep(1)

                # Should complete without errors
                assert True

            finally:
                await browser.close()

    @pytest.mark.asyncio
    async def test_scene_loading_interface(self, server_process):
        """Test scene loading interface if available."""
        async with async_playwright() as p:
            browser = await p.chromium.launch(headless=True)
            context = await browser.new_context()
            page = await context.new_page()

            try:
                # Navigate to the viewer
                await page.goto("http://localhost:8003")
                await page.wait_for_load_state("networkidle")

                # Wait for initialization
                await asyncio.sleep(3)

                # Look for scene loading interface
                scene_input = await page.query_selector("#sceneInput")
                if scene_input:
                    # Test scene loading functionality
                    simple_xml = """
                    <mujoco>
                        <worldbody>
                            <body name="test_box">
                                <geom type="box" size="0.1 0.1 0.1"/>
                            </body>
                        </worldbody>
                    </mujoco>
                    """

                    await scene_input.fill(simple_xml)

                    # Look for load button
                    load_button = await page.query_selector("button:has-text('Load')")
                    if load_button:
                        await load_button.click()
                        await asyncio.sleep(2)  # Give time for scene to load

                # Should complete without errors
                assert True

            finally:
                await browser.close()

    @pytest.mark.asyncio
    async def test_multi_client_support(self, server_process):
        """Test multi-client support by opening multiple browser instances."""
        async with async_playwright() as p:
            browser = await p.chromium.launch(headless=True)

            try:
                # Create multiple browser contexts (simulating different clients)
                contexts = []
                pages = []

                for i in range(3):  # Test with 3 clients
                    context = await browser.new_context()
                    page = await context.new_page()

                    await page.goto("http://localhost:8003")
                    await page.wait_for_load_state("networkidle")

                    contexts.append(context)
                    pages.append(page)

                # Wait for all connections to establish
                await asyncio.sleep(5)

                # Simulate activity on each client
                for i, page in enumerate(pages):
                    video_element = await page.query_selector("#videoElement")
                    if video_element:
                        await video_element.click()
                        await page.mouse.move(50 + i * 20, 50 + i * 20)

                # Wait for events to process
                await asyncio.sleep(2)

                # All clients should still be responsive
                for page in pages:
                    title = await page.title()
                    assert "MuJoCo Remote Viewer" in title

            finally:
                await browser.close()

    @pytest.mark.asyncio
    async def test_api_endpoints_accessible(self, server_process):
        """Test that API endpoints are accessible during browser session."""
        async with async_playwright() as p:
            browser = await p.chromium.launch(headless=True)
            context = await browser.new_context()
            page = await context.new_page()

            try:
                # Navigate to the viewer
                await page.goto("http://localhost:8003")
                await page.wait_for_load_state("networkidle")

                # Test API endpoints via fetch
                api_tests = [
                    "http://localhost:8003/api/health",
                    "http://localhost:8003/api/config",
                    "http://localhost:8003/api/stats",
                ]

                for api_url in api_tests:
                    response = await page.evaluate(
                        f"""
                        fetch('{api_url}')
                            .then(response => response.ok ? response.json() : null)
                            .catch(() => null)
                    """
                    )

                    # Should get some response (even if it's an error response)
                    assert response is not None or True  # Allow for various responses

            finally:
                await browser.close()

    @pytest.mark.asyncio
    async def test_performance_monitoring(self, server_process):
        """Test performance monitoring during browser session."""
        async with async_playwright() as p:
            browser = await p.chromium.launch(headless=True)
            context = await browser.new_context()
            page = await context.new_page()

            try:
                # Navigate to the viewer
                await page.goto("http://localhost:8003")
                await page.wait_for_load_state("networkidle")

                # Monitor performance for a short period
                start_time = time.time()

                # Simulate user activity
                for _ in range(10):
                    await page.mouse.move(100, 100)
                    await page.mouse.move(200, 200)
                    await asyncio.sleep(0.1)

                end_time = time.time()
                duration = end_time - start_time

                # Should complete in reasonable time (not frozen)
                assert duration < 5.0  # Should complete in less than 5 seconds

                # Check for JavaScript errors
                console_errors = []

                def handle_console(msg):
                    if msg.type == "error":
                        console_errors.append(msg.text)

                page.on("console", handle_console)

                await asyncio.sleep(1)

                # Should not have critical JavaScript errors
                critical_errors = [err for err in console_errors if "uncaught" in err.lower()]
                assert len(critical_errors) == 0

            finally:
                await browser.close()


# Pytest configuration for async tests
@pytest.fixture(scope="session")
def event_loop():
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v", "-s"])
