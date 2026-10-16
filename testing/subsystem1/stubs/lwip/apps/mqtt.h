/* Stand-in for the lwIP MQTT client, for PC tests only.
 * It records what comms.c asks for and lets the test play the broker. */
#include <stdint.h>
#include <string.h>
typedef uint32_t u32_t; typedef uint16_t u16_t; typedef uint8_t u8_t; typedef int err_t;
#define ERR_OK 0
#define ERR_BUF (-2)
#define MQTT_DATA_FLAG_LAST 1
#define MQTT_CONNECT_ACCEPTED 0
typedef int mqtt_connection_status_t;
typedef struct { int x; } mqtt_client_t;
struct mqtt_connect_client_info_t { const char *client_id; const char *client_user; const char *client_pass; int keep_alive; };
typedef void (*mqtt_connection_cb_t)(mqtt_client_t *, void *, mqtt_connection_status_t);
typedef void (*mqtt_request_cb_t)(void *, err_t);
typedef void (*mqtt_incoming_publish_cb_t)(void *, const char *, u32_t);
typedef void (*mqtt_incoming_data_cb_t)(void *, const u8_t *, u16_t, u8_t);

static mqtt_client_t fk_client;
static mqtt_connection_cb_t fk_conn_cb;
static mqtt_request_cb_t fk_pub_cb, fk_sub_cb;
static void *fk_pub_arg;
static int fk_connects = 0, fk_disconnects = 0, fk_subscribes = 0, fk_publishes = 0;
static uint32_t fk_connect_times[64];
static char fk_pub_topic[64];
static char fk_pub_payload[300];
static int fk_pub_len = 0;
static err_t fk_publish_ret = ERR_OK;    /* set to an error to refuse publishes */

/* fk_lock_depth / fk_unlocked_calls come from fake_port.h (included first) */
#define FK_NEED_LOCK() do { if (fk_lock_depth != 1) { fk_unlocked_calls++; } } while (0)
static inline mqtt_client_t *mqtt_client_new(void) { FK_NEED_LOCK(); return &fk_client; }
static inline void mqtt_set_inpub_callback(mqtt_client_t *c, mqtt_incoming_publish_cb_t a,
                                           mqtt_incoming_data_cb_t b, void *arg)
{ FK_NEED_LOCK(); (void)c; (void)a; (void)b; (void)arg; }
static inline err_t mqtt_client_connect(mqtt_client_t *c, const void *ip, int p,
        mqtt_connection_cb_t cb, void *arg, const struct mqtt_connect_client_info_t *ci)
{ FK_NEED_LOCK(); (void)c; (void)ip; (void)p; (void)arg; (void)ci; fk_conn_cb = cb;
  if (fk_connects < 64) { fk_connect_times[fk_connects] = fk_now_ms; }
  fk_connects++; return ERR_OK; }
static inline void mqtt_disconnect(mqtt_client_t *c) { FK_NEED_LOCK(); (void)c; fk_disconnects++; }
static inline err_t mqtt_subscribe(mqtt_client_t *c, const char *t, int q, mqtt_request_cb_t cb, void *arg)
{ FK_NEED_LOCK(); (void)c; (void)t; (void)q; (void)arg; fk_sub_cb = cb; fk_subscribes++; return ERR_OK; }
static inline err_t mqtt_publish(mqtt_client_t *c, const char *t, const void *p, u16_t l,
        int q, int r, mqtt_request_cb_t cb, void *arg)
{ FK_NEED_LOCK(); (void)c; (void)q; (void)r;
  if (fk_publish_ret != ERR_OK) { return fk_publish_ret; }
  strncpy(fk_pub_topic, t, sizeof fk_pub_topic - 1);
  memcpy(fk_pub_payload, p, l); fk_pub_payload[l] = 0; fk_pub_len = l;
  fk_pub_cb = cb; fk_pub_arg = arg; fk_publishes++; return ERR_OK; }