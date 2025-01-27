#include <driver/i2s.h>
#include <opus.h>

#include "main.h"

#define OPUS_OUT_BUFFER_SIZE 1276 // 1276 bytes is recommended by opus_encode
#define SAMPLE_RATE 8000
#define MIC_CHANNELS 1
#define SPEAK_CHANNELS 2

#define BUFFER_SAMPLES (320)
#define BUFFER_SAMPLES_CNT (BUFFER_SAMPLES / 2)

#define OPUS_ENCODER_BITRATE 30000
#define OPUS_ENCODER_COMPLEXITY 0

static esp_codec_dev_handle_t play_dev_handle;
static esp_codec_dev_handle_t record_dev_handle;

void wheatly_init_audio_capture()
{
  bsp_codec_mute_set(true);
  bsp_codec_mute_set(false);
  bsp_codec_volume_set(100, NULL);

  play_dev_handle = bsp_codec_speaker_get();
  record_dev_handle = bsp_codec_microphone_get();
  return;
}

opus_int16 *output_stereo_buffer = NULL;
opus_int16 *output_mono_buffer = NULL;
OpusDecoder *opus_decoder = NULL;

void wheatly_init_audio_decoder()
{
  int decoder_error = 0;
  opus_decoder = opus_decoder_create(SAMPLE_RATE, SPEAK_CHANNELS, &decoder_error);
  if (decoder_error != OPUS_OK)
  {
    printf("Failed to create OPUS decoder");
    return;
  }
  output_stereo_buffer = (opus_int16 *)malloc(BUFFER_SAMPLES * sizeof(opus_int16));
  output_mono_buffer = (opus_int16 *)malloc(BUFFER_SAMPLES_CNT * sizeof(opus_int16));
}

void wheatly_audio_decode(uint8_t *data, size_t size)
{
  // Stereo decoded size
  int decoded_size =
      opus_decode(opus_decoder, data, size, output_stereo_buffer, BUFFER_SAMPLES, 0);

  // if (size > 26)
  // {
  //   printf("Audio Speaking Receive: size: %d, decode size: %d\r\n", size, decoded_size);
  // }

  for (int i = 0; i < BUFFER_SAMPLES_CNT; i++)
  {
    output_mono_buffer[i] = (output_stereo_buffer[i * 2] + output_stereo_buffer[i * 2 + 1]) / 2;
  }

  if (decoded_size > 0)
  {
    esp_codec_dev_write(play_dev_handle, output_mono_buffer, BUFFER_SAMPLES_CNT * sizeof(opus_int16));
  }
}

OpusEncoder *opus_encoder = NULL;
opus_int16 *encoder_input_buffer = NULL;
uint8_t *encoder_output_buffer = NULL;

void wheatly_init_audio_encoder()
{
  int encoder_error;
  opus_encoder = opus_encoder_create(SAMPLE_RATE, MIC_CHANNELS, OPUS_APPLICATION_VOIP,
                                     &encoder_error);
  if (encoder_error != OPUS_OK)
  {
    printf("Failed to create OPUS encoder");
    return;
  }

  if (opus_encoder_init(opus_encoder, SAMPLE_RATE, MIC_CHANNELS, OPUS_APPLICATION_VOIP) !=
      OPUS_OK)
  {
    printf("Failed to initialize OPUS encoder");
    return;
  }

  opus_encoder_ctl(opus_encoder, OPUS_SET_BITRATE(OPUS_ENCODER_BITRATE));
  opus_encoder_ctl(opus_encoder, OPUS_SET_COMPLEXITY(OPUS_ENCODER_COMPLEXITY));
  opus_encoder_ctl(opus_encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
  encoder_input_buffer = (opus_int16 *)malloc(BUFFER_SAMPLES);
  encoder_output_buffer = (uint8_t *)malloc(OPUS_OUT_BUFFER_SIZE);
}

void wheatly_send_audio(PeerConnection *peer_connection)
{

  esp_codec_dev_read(record_dev_handle, encoder_input_buffer, BUFFER_SAMPLES);

  auto encoded_size =
      opus_encode(opus_encoder, encoder_input_buffer, BUFFER_SAMPLES_CNT,
                  encoder_output_buffer, OPUS_OUT_BUFFER_SIZE);

  // printf("Audio Send: size: %d, encoded_size : %ld\r\n", BUFFER_SAMPLES, encoded_size);
  peer_connection_send_audio(peer_connection, encoder_output_buffer,
                             encoded_size);
}
