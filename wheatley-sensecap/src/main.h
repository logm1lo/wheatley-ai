#include <peer.h>
#include "sensecap-watcher.h"
extern "C"
{
#include "ui.h"
}

#define LIVEKIT_LOG "live-kit"
#define MAX_HTTP_OUTPUT_BUFFER 2048

// Wifi Functions [Done]
void wheatly_wifi(void);
void wheatly_wifi_init(void);

// Audio Init Functions [Done]
void wheatly_init_audio_capture(void);
void wheatly_init_audio_decoder(void);
void wheatly_init_audio_encoder();

// Audio Functions [Done]
void wheatly_send_audio(PeerConnection *peer_connection);
void wheatly_audio_decode(uint8_t *data, size_t size);

// WebRTC Functions [Done]
PeerConnection *lk_create_peer_connection(int isPublisher);
void lk_populate_answer(char *answer, size_t answer_size, int include_audio);
void lk_publisher_peer_connection_task(void *user_data);
void lk_subscriber_peer_connection_task(void *user_data);

// Service Functions [Done]
void lk_websocket(const char *url, const char *token);

// CMD Functions [Done]
int cmd_init(void);
