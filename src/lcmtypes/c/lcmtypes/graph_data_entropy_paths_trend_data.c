// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "lcmtypes/graph_data_entropy_paths_trend_data.h"

static int __graph_data_entropy_paths_trend_data_hash_computed;
static uint64_t __graph_data_entropy_paths_trend_data_hash;

uint64_t __graph_data_entropy_paths_trend_data_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __graph_data_entropy_paths_trend_data_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = __graph_data_entropy_paths_trend_data_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0xf1223a9b5bea0642LL
         + __int32_t_hash_recursive(&cp)
         + __graph_data_entropy_path_trend_data_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __graph_data_entropy_paths_trend_data_get_hash(void)
{
    if (!__graph_data_entropy_paths_trend_data_hash_computed) {
        __graph_data_entropy_paths_trend_data_hash = (int64_t)__graph_data_entropy_paths_trend_data_hash_recursive(NULL);
        __graph_data_entropy_paths_trend_data_hash_computed = 1;
    }

    return __graph_data_entropy_paths_trend_data_hash;
}

int __graph_data_entropy_paths_trend_data_encode_array(void *buf, int offset, int maxlen, const graph_data_entropy_paths_trend_data *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].num_path_), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __graph_data_entropy_path_trend_data_encode_array(buf, offset + pos, maxlen - pos, p[element].entropy_paths_, p[element].num_path_);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int graph_data_entropy_paths_trend_data_encode(void *buf, int offset, int maxlen, const graph_data_entropy_paths_trend_data *p)
{
    int pos = 0, thislen;
    int64_t hash = __graph_data_entropy_paths_trend_data_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __graph_data_entropy_paths_trend_data_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __graph_data_entropy_paths_trend_data_encoded_array_size(const graph_data_entropy_paths_trend_data *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int32_t_encoded_array_size(&(p[element].num_path_), 1);

        size += __graph_data_entropy_path_trend_data_encoded_array_size(p[element].entropy_paths_, p[element].num_path_);

    }
    return size;
}

int graph_data_entropy_paths_trend_data_encoded_size(const graph_data_entropy_paths_trend_data *p)
{
    return 8 + __graph_data_entropy_paths_trend_data_encoded_array_size(p, 1);
}

int __graph_data_entropy_paths_trend_data_decode_array(const void *buf, int offset, int maxlen, graph_data_entropy_paths_trend_data *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].num_path_), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        p[element].entropy_paths_ = (graph_data_entropy_path_trend_data*) lcm_malloc(sizeof(graph_data_entropy_path_trend_data) * p[element].num_path_);
        thislen = __graph_data_entropy_path_trend_data_decode_array(buf, offset + pos, maxlen - pos, p[element].entropy_paths_, p[element].num_path_);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __graph_data_entropy_paths_trend_data_decode_array_cleanup(graph_data_entropy_paths_trend_data *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int32_t_decode_array_cleanup(&(p[element].num_path_), 1);

        __graph_data_entropy_path_trend_data_decode_array_cleanup(p[element].entropy_paths_, p[element].num_path_);
        if (p[element].entropy_paths_) free(p[element].entropy_paths_);

    }
    return 0;
}

int graph_data_entropy_paths_trend_data_decode(const void *buf, int offset, int maxlen, graph_data_entropy_paths_trend_data *p)
{
    int pos = 0, thislen;
    int64_t hash = __graph_data_entropy_paths_trend_data_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __graph_data_entropy_paths_trend_data_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int graph_data_entropy_paths_trend_data_decode_cleanup(graph_data_entropy_paths_trend_data *p)
{
    return __graph_data_entropy_paths_trend_data_decode_array_cleanup(p, 1);
}

int __graph_data_entropy_paths_trend_data_clone_array(const graph_data_entropy_paths_trend_data *p, graph_data_entropy_paths_trend_data *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int32_t_clone_array(&(p[element].num_path_), &(q[element].num_path_), 1);

        q[element].entropy_paths_ = (graph_data_entropy_path_trend_data*) lcm_malloc(sizeof(graph_data_entropy_path_trend_data) * q[element].num_path_);
        __graph_data_entropy_path_trend_data_clone_array(p[element].entropy_paths_, q[element].entropy_paths_, p[element].num_path_);

    }
    return 0;
}

graph_data_entropy_paths_trend_data *graph_data_entropy_paths_trend_data_copy(const graph_data_entropy_paths_trend_data *p)
{
    graph_data_entropy_paths_trend_data *q = (graph_data_entropy_paths_trend_data*) malloc(sizeof(graph_data_entropy_paths_trend_data));
    __graph_data_entropy_paths_trend_data_clone_array(p, q, 1);
    return q;
}

void graph_data_entropy_paths_trend_data_destroy(graph_data_entropy_paths_trend_data *p)
{
    __graph_data_entropy_paths_trend_data_decode_array_cleanup(p, 1);
    free(p);
}

int graph_data_entropy_paths_trend_data_publish(lcm_t *lc, const char *channel, const graph_data_entropy_paths_trend_data *p)
{
      int max_data_size = graph_data_entropy_paths_trend_data_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = graph_data_entropy_paths_trend_data_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _graph_data_entropy_paths_trend_data_subscription_t {
    graph_data_entropy_paths_trend_data_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void graph_data_entropy_paths_trend_data_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    graph_data_entropy_paths_trend_data p;
    memset(&p, 0, sizeof(graph_data_entropy_paths_trend_data));
    status = graph_data_entropy_paths_trend_data_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding graph_data_entropy_paths_trend_data!!!\n", status);
        return;
    }

    graph_data_entropy_paths_trend_data_subscription_t *h = (graph_data_entropy_paths_trend_data_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    graph_data_entropy_paths_trend_data_decode_cleanup (&p);
}

graph_data_entropy_paths_trend_data_subscription_t* graph_data_entropy_paths_trend_data_subscribe (lcm_t *lcm,
                    const char *channel,
                    graph_data_entropy_paths_trend_data_handler_t f, void *userdata)
{
    graph_data_entropy_paths_trend_data_subscription_t *n = (graph_data_entropy_paths_trend_data_subscription_t*)
                       malloc(sizeof(graph_data_entropy_paths_trend_data_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 graph_data_entropy_paths_trend_data_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg graph_data_entropy_paths_trend_data LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int graph_data_entropy_paths_trend_data_subscription_set_queue_capacity (graph_data_entropy_paths_trend_data_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int graph_data_entropy_paths_trend_data_unsubscribe(lcm_t *lcm, graph_data_entropy_paths_trend_data_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe graph_data_entropy_paths_trend_data_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

