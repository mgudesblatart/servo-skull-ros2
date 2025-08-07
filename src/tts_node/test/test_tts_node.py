import unittest
from unittest.mock import patch, MagicMock
import array
from tts_node.tts_node import TTSNode
from std_msgs.msg import String

class TestTTSNode(unittest.TestCase):
    def setUp(self):
        import rclpy
        rclpy.init()

    def tearDown(self):
        import rclpy
        rclpy.shutdown()

    @patch('tts_node.tts_node.PiperVoice')
    def test_tts_callback_publishes_audio(self, MockPiperVoice):
        # Arrange
        mock_voice = MagicMock()
        # Simulate PiperVoice.synthesize yielding a chunk with audio_int16_bytes
        mock_chunk = MagicMock()
        mock_chunk.audio_int16_bytes = array.array('h', [1, 2, 3, 4]).tobytes()
        mock_voice.synthesize.return_value = [mock_chunk]
        MockPiperVoice.load.return_value = mock_voice

        node = TTSNode()
        node.audio_pub = MagicMock()

        # Act
        msg = String()
        msg.data = "test"
        node.tts_callback(msg)

        # Assert
        node.audio_pub.publish.assert_called_once()
        published_msg = node.audio_pub.publish.call_args[0][0]
        self.assertEqual(list(published_msg.data), [1, 2, 3, 4])

    @patch('tts_node.tts_node.PiperVoice')
    def test_tts_callback_empty_input(self, MockPiperVoice):
        # Arrange
        mock_voice = MagicMock()
        MockPiperVoice.load.return_value = mock_voice

        node = TTSNode()
        node.audio_pub = MagicMock()

        # Act
        msg = String()
        msg.data = ""
        node.tts_callback(msg)

        # Assert
        node.audio_pub.publish.assert_not_called()

    @patch('tts_node.tts_node.PiperVoice')
    def test_tts_callback_multiple_chunks(self, MockPiperVoice):
        # Arrange
        mock_voice = MagicMock()
        mock_chunk1 = MagicMock()
        mock_chunk1.audio_int16_bytes = array.array('h', [1, 2]).tobytes()
        mock_chunk2 = MagicMock()
        mock_chunk2.audio_int16_bytes = array.array('h', [3, 4]).tobytes()
        mock_voice.synthesize.return_value = [mock_chunk1, mock_chunk2]
        MockPiperVoice.load.return_value = mock_voice

        node = TTSNode()
        node.audio_pub = MagicMock()

        # Act
        msg = String()
        msg.data = "test"
        node.tts_callback(msg)

        # Assert
        self.assertEqual(node.audio_pub.publish.call_count, 2)
        published_msgs = [call[0][0] for call in node.audio_pub.publish.call_args_list]
        self.assertEqual(list(published_msgs[0].data), [1, 2])
        self.assertEqual(list(published_msgs[1].data), [3, 4])

    @patch('tts_node.tts_node.PiperVoice')
    def test_tts_callback_piper_exception(self, MockPiperVoice):
        # Arrange
        mock_voice = MagicMock()
        mock_voice.synthesize.side_effect = Exception("synthesis fail")
        MockPiperVoice.load.return_value = mock_voice

        node = TTSNode()
        node.audio_pub = MagicMock()

        msg = String()
        msg.data = "test"
        # Act
        # Should not raise, just log error
        node.tts_callback(msg)

        # Assert
        node.audio_pub.publish.assert_not_called()

    @patch('tts_node.tts_node.PiperVoice')
    def test_model_load_failure(self, MockPiperVoice):
        # Arrange
        MockPiperVoice.load.side_effect = Exception("model fail")

        # Act
        node = TTSNode()

        # Assert
        self.assertIsNone(node.voice)

if __name__ == '__main__':
    unittest.main()
