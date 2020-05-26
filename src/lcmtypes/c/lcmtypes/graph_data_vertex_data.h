// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#ifndef _graph_data_vertex_data_h
#define _graph_data_vertex_data_h

#include <stdint.h>
#include <stdlib.h>
#include <lcm/lcm_coretypes.h>
#include <lcm/lcm.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _graph_data_vertex_data graph_data_vertex_data;
struct _graph_data_vertex_data
{
    int64_t    idx_;
    double     ig_;
    double     p_;
    int8_t     isROIs_;
    int8_t     isNZIG_;
    int8_t     isSamples_;
};

/**
 * Create a deep copy of a graph_data_vertex_data.
 * When no longer needed, destroy it with graph_data_vertex_data_destroy()
 */
graph_data_vertex_data* graph_data_vertex_data_copy(const graph_data_vertex_data* to_copy);

/**
 * Destroy an instance of graph_data_vertex_data created by graph_data_vertex_data_copy()
 */
void graph_data_vertex_data_destroy(graph_data_vertex_data* to_destroy);

/**
 * Identifies a single subscription.  This is an opaque data type.
 */
typedef struct _graph_data_vertex_data_subscription_t graph_data_vertex_data_subscription_t;

/**
 * Prototype for a callback function invoked when a message of type
 * graph_data_vertex_data is received.
 */
typedef void(*graph_data_vertex_data_handler_t)(
    const lcm_recv_buf_t *rbuf, const char *channel,
    const graph_data_vertex_data *msg, void *userdata);

/**
 * Publish a message of type graph_data_vertex_data using LCM.
 *
 * @param lcm The LCM instance to publish with.
 * @param channel The channel to publish on.
 * @param msg The message to publish.
 * @return 0 on success, <0 on error.  Success means LCM has transferred
 * responsibility of the message data to the OS.
 */
int graph_data_vertex_data_publish(lcm_t *lcm, const char *channel, const graph_data_vertex_data *msg);

/**
 * Subscribe to messages of type graph_data_vertex_data using LCM.
 *
 * @param lcm The LCM instance to subscribe with.
 * @param channel The channel to subscribe to.
 * @param handler The callback function invoked by LCM when a message is
 *     received. This function is invoked by LCM during calls to lcm_handle()
 *     and lcm_handle_timeout().
 * @param userdata An opaque pointer passed to @p handler when it is invoked.
 * @return 0 on success, <0 if an error occured
 */
graph_data_vertex_data_subscription_t* graph_data_vertex_data_subscribe(
    lcm_t *lcm, const char *channel, graph_data_vertex_data_handler_t handler, void *userdata);

/**
 * Removes and destroys a subscription created by graph_data_vertex_data_subscribe()
 */
int graph_data_vertex_data_unsubscribe(lcm_t *lcm, graph_data_vertex_data_subscription_t* hid);

/**
 * Sets the queue capacity for a subscription.
 * Some LCM providers (e.g., the default multicast provider) are implemented
 * using a background receive thread that constantly revceives messages from
 * the network.  As these messages are received, they are buffered on
 * per-subscription queues until dispatched by lcm_handle().  This function
 * how many messages are queued before dropping messages.
 *
 * @param subs the subscription to modify.
 * @param num_messages The maximum number of messages to queue
 *  on the subscription.
 * @return 0 on success, <0 if an error occured
 */
int graph_data_vertex_data_subscription_set_queue_capacity(
    graph_data_vertex_data_subscription_t* subs, int num_messages);

/**
 * Encode a message of type graph_data_vertex_data into binary form.
 *
 * @param buf The output buffer.
 * @param offset Encoding starts at this byte offset into @p buf.
 * @param maxlen Maximum number of bytes to write.  This should generally
 *               be equal to graph_data_vertex_data_encoded_size().
 * @param msg The message to encode.
 * @return The number of bytes encoded, or <0 if an error occured.
 */
int graph_data_vertex_data_encode(void *buf, int offset, int maxlen, const graph_data_vertex_data *p);

/**
 * Decode a message of type graph_data_vertex_data from binary form.
 * When decoding messages containing strings or variable-length arrays, this
 * function may allocate memory.  When finished with the decoded message,
 * release allocated resources with graph_data_vertex_data_decode_cleanup().
 *
 * @param buf The buffer containing the encoded message
 * @param offset The byte offset into @p buf where the encoded message starts.
 * @param maxlen The maximum number of bytes to read while decoding.
 * @param msg Output parameter where the decoded message is stored
 * @return The number of bytes decoded, or <0 if an error occured.
 */
int graph_data_vertex_data_decode(const void *buf, int offset, int maxlen, graph_data_vertex_data *msg);

/**
 * Release resources allocated by graph_data_vertex_data_decode()
 * @return 0
 */
int graph_data_vertex_data_decode_cleanup(graph_data_vertex_data *p);

/**
 * Check how many bytes are required to encode a message of type graph_data_vertex_data
 */
int graph_data_vertex_data_encoded_size(const graph_data_vertex_data *p);

// LCM support functions. Users should not call these
int64_t __graph_data_vertex_data_get_hash(void);
uint64_t __graph_data_vertex_data_hash_recursive(const __lcm_hash_ptr *p);
int __graph_data_vertex_data_encode_array(
    void *buf, int offset, int maxlen, const graph_data_vertex_data *p, int elements);
int __graph_data_vertex_data_decode_array(
    const void *buf, int offset, int maxlen, graph_data_vertex_data *p, int elements);
int __graph_data_vertex_data_decode_array_cleanup(graph_data_vertex_data *p, int elements);
int __graph_data_vertex_data_encoded_array_size(const graph_data_vertex_data *p, int elements);
int __graph_data_vertex_data_clone_array(const graph_data_vertex_data *p, graph_data_vertex_data *q, int elements);

#ifdef __cplusplus
}
#endif

#endif
