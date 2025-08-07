#!/usr/bin/env python3

import unittest
import pytest
import numpy as np
from unittest.mock import MagicMock, patch
import os
import sys
import threading
import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String

# Import the class to test
from stt_node.stt_node import STTNode, MODEL_DIR


@pytest.fixture
def mock_sherpa_onnx():
    """Mock sherpa_onnx to avoid loading actual models and processing."""
    with patch("stt_node.stt_node.sherpa_onnx") as mock:
        # Mock the stream object
        mock_stream = MagicMock()

        # Mock the recognizer object and its methods
        mock_recognizer = MagicMock()
        mock_recognizer.create_stream.return_value = mock_stream
        mock_recognizer.is_ready.side_effect = [True, False]
        mock_recognizer.is_endpoint.side_effect = [
            False,
            False,
            True,
        ]  # Third call returns endpoint detected
        mock_recognizer.get_result.return_value = "test transcript"

        # Configure the main class - this mocks from_transducer
        mock.OnlineRecognizer = MagicMock()
        mock.OnlineRecognizer.from_transducer = MagicMock(return_value=mock_recognizer)

        yield mock


@pytest.fixture
def mock_get_package_share_directory():
    """Mock get_package_share_directory to avoid filesystem dependencies."""
    with patch("stt_node.stt_node.get_package_share_directory") as mock:
        mock.return_value = "/mock/path/to/package"
        yield mock


@pytest.mark.usefixtures("mock_get_package_share_directory", "mock_sherpa_onnx")
class TestSTTNode(unittest.TestCase):

    def setUp(self):
        # Initialize ROS
        rclpy.init()

    def tearDown(self):
        # Shutdown ROS
        rclpy.shutdown()

    def test_node_initialization(self):
        """Test that the node initializes properly using fixtures."""
        # The fixtures are automatically applied before the test runs
        # No need to set up the mocks again, they're already configured in the fixtures
        # Create node
        node = STTNode()

        # Assert node properties are set up correctly
        self.assertIsNotNone(node.subscription)
        self.assertIsNotNone(node.transcript_publisher)
        self.assertIsNotNone(node.recognizer)
        self.assertIsNotNone(node.stream)
        self.assertEqual(node.pcm_buffer.size, 0)
        self.assertEqual(node.last_result, "")
        self.assertEqual(node.output, [])
        self.assertIsNone(node.worker_thread)

        # Clean up
        node.destroy_node()

    def test_start_worker(self):
        """Test that the worker thread starts properly."""
        # Create node
        node = STTNode()

        # Check that worker thread doesn't exist yet
        self.assertIsNone(node.worker_thread)

        # Start worker thread
        node.start_worker()

        # Check that worker thread exists now and is running
        self.assertIsNotNone(node.worker_thread)
        self.assertTrue(node.worker_thread.is_alive())

        # Clean up
        node.destroy_node()

    @patch(
        "stt_node.stt_node.resample_poly"
    )  # Mock resample_poly to avoid scipy dependency
    def test_audio_processing_flow(self, mock_resample):
        """Test the full audio processing flow."""
        # Mock resample_poly to return the input (no resampling)
        mock_resample.side_effect = lambda x, up, down: x

        # Create node and patch audio_queue to simulate data
        node = STTNode()

        # Get the actual stream used by the node (from the fixture's mock_recognizer)
        stream = node.stream

        # Create a test message
        test_audio = np.ones(1000, dtype=np.int16).tobytes()
        msg = UInt8MultiArray()
        msg.data = list(test_audio)

        # Set up publisher callback capture
        published_msgs = []

        def mock_publish(msg):
            published_msgs.append(msg)

        # Replace publish method with our mock
        node.transcript_publisher.publish = mock_publish

        # Call audio callback to add data to queue
        node.audio_callback(msg)

        node.recognizer.is_ready.side_effect = [True, False]
        node.recognizer.is_endpoint.side_effect = [
            True,
            False,
        ]

        try:
            node._audio_worker(once=True)
        except Exception:
            pass  # Ignore errors from empty queue, etc.

            # Assert the recognizer methods were called
        stream.accept_waveform.assert_called()
        node.recognizer.decode_stream.assert_called()
        node.recognizer.get_result.assert_called()
        node.recognizer.is_endpoint.assert_called()
        node.recognizer.reset.assert_called()

        # Check if a message was published
        self.assertEqual(len(published_msgs), 1)
        self.assertEqual(published_msgs[0].data, "test transcript")

        # Clean up
        node.destroy_node()

    def test_audio_worker_handles_queue_empty(self):
        """Test that the audio worker exits cleanly on queue.Empty when once=True."""
        node = STTNode()
        # Don't put anything in the queue
        node.recognizer.is_ready.side_effect = [False]
        node.recognizer.is_endpoint.side_effect = [False]
        try:
            node._audio_worker(once=True)
        except Exception as e:
            self.fail(f"audio_worker raised unexpected exception: {e}")
        node.destroy_node()

    def test_audio_worker_handles_accept_waveform_exception(self):
        """Test that the audio worker logs and continues if accept_waveform raises."""
        node = STTNode()
        node.recognizer.is_ready.side_effect = [True, False]
        node.recognizer.is_endpoint.side_effect = [True, False]
        # Put a chunk in the queue
        test_audio = np.ones(1000, dtype=np.int16).tobytes()
        msg = UInt8MultiArray()
        msg.data = list(test_audio)
        node.audio_callback(msg)
        # Patch accept_waveform to raise
        node.stream.accept_waveform.side_effect = RuntimeError("fail")
        # Patch logger to capture error
        with patch.object(node, 'get_logger') as mock_logger:
            node._audio_worker(once=True)
            # Check that error was logged
            assert mock_logger().error.called
        node.destroy_node()

    def test_audio_worker_handles_publish_exception(self):
        """Test that the audio worker logs and continues if publish raises."""
        node = STTNode()
        test_audio = np.ones(1000, dtype=np.int16).tobytes()
        msg = UInt8MultiArray()
        msg.data = list(test_audio)
        node.audio_callback(msg)
        # Patch recognizer to always endpoint
        node.recognizer.is_ready.side_effect = [True, False]
        node.recognizer.is_endpoint.side_effect = [True, False]
        # Patch transcript_publisher with a mock
        node.transcript_publisher = MagicMock()
        node.transcript_publisher.publish.side_effect = RuntimeError("fail")
        with patch.object(node, 'get_logger') as mock_logger:
            node._audio_worker(once=True)
            assert mock_logger().error.called
        node.destroy_node()

    def test_audio_worker_multiple_chunks(self):
        """Test that the audio worker processes multiple audio chunks in sequence."""
        node = STTNode()
        # Patch recognizer to endpoint after each chunk

        node.recognizer.get_result.return_value = "test transcript"
        published_msgs = []
        node.transcript_publisher.publish = lambda msg: published_msgs.append(msg)
        # Add two chunks
        for _ in range(2):
            test_audio = np.ones(1000, dtype=np.int16).tobytes()
            msg = UInt8MultiArray()
            msg.data = list(test_audio)
            node.audio_callback(msg)
        # Run worker twice
        for _ in range(2):
            node.recognizer.is_ready.side_effect = [True, False]
            node.recognizer.is_endpoint.side_effect = [True, False]
            node._audio_worker(once=True)
        self.assertEqual(len(published_msgs), 2)
        node.destroy_node()

    def test_model_init_failure(self):
        """Test that the node handles model init failure gracefully."""
        with patch("stt_node.stt_node.sherpa_onnx.OnlineRecognizer.from_transducer", side_effect=RuntimeError("model fail")):
            try:
                node = STTNode()
            except Exception as e:
                self.assertIn("model fail", str(e))
            else:
                self.fail("STTNode did not raise on model init failure")


@pytest.mark.parametrize(
    "mock_audio_data,expected_calls",
    [
        (np.zeros(1000, dtype=np.int16), 1),  # Test with silence
        (np.ones(2000, dtype=np.int16) * 16000, 1),  # Test with loud audio
    ],
)
def test_audio_callback_parameterized(
    mock_audio_data,
    expected_calls,
    mock_get_package_share_directory,
    mock_sherpa_onnx,
):
    """Test audio callback with different input data."""
    # Initialize ROS
    rclpy.init()

    # Create node
    node = STTNode()

    # Track number of items added to queue
    original_queue_put = node.audio_queue.put
    put_count = 0

    def count_puts(item):
        nonlocal put_count
        put_count += 1
        return original_queue_put(item)

    node.audio_queue.put = count_puts

    # Create test message
    msg = UInt8MultiArray()
    msg.data = list(mock_audio_data.tobytes())

    # Call audio callback
    node.audio_callback(msg)

    # Verify queue operations
    assert put_count == expected_calls

    # Clean up
    node.destroy_node()
    rclpy.shutdown()


def test_main():
    """Test main function initializes and shuts down properly."""
    with patch("rclpy.init"), patch("rclpy.spin"), patch("rclpy.shutdown"), patch(
        "stt_node.stt_node.STTNode"
    ) as mock_node:

        mock_instance = MagicMock()
        mock_node.return_value = mock_instance

        from stt_node.stt_node import main

        main()

        # Verify the expected methods were called
        mock_node.assert_called_once()
        mock_instance.start_worker.assert_called_once()
        mock_instance.destroy_node.assert_called_once()


if __name__ == "__main__":
    pytest.main(["-xvs", __file__])
