// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "lcmtypes/random_graph_data_agent_data.h"

static int __random_graph_data_agent_data_hash_computed;
static uint64_t __random_graph_data_agent_data_hash;

uint64_t __random_graph_data_agent_data_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __random_graph_data_agent_data_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = __random_graph_data_agent_data_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x48dd73df93cd94b7LL
         + __int32_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __random_graph_data_agent_data_get_hash(void)
{
    if (!__random_graph_data_agent_data_hash_computed) {
        __random_graph_data_agent_data_hash = (int64_t)__random_graph_data_agent_data_hash_recursive(NULL);
        __random_graph_data_agent_data_hash_computed = 1;
    }

    return __random_graph_data_agent_data_hash;
}

int __random_graph_data_agent_data_encode_array(void *buf, int offset, int maxlen, const random_graph_data_agent_data *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].init_pos_), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].idx_), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int random_graph_data_agent_data_encode(void *buf, int offset, int maxlen, const random_graph_data_agent_data *p)
{
    int pos = 0, thislen;
    int64_t hash = __random_graph_data_agent_data_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __random_graph_data_agent_data_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __random_graph_data_agent_data_encoded_array_size(const random_graph_data_agent_data *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int32_t_encoded_array_size(&(p[element].init_pos_), 1);

        size += __int32_t_encoded_array_size(&(p[element].idx_), 1);

    }
    return size;
}

int random_graph_data_agent_data_encoded_size(const random_graph_data_agent_data *p)
{
    return 8 + __random_graph_data_agent_data_encoded_array_size(p, 1);
}

int __random_graph_data_agent_data_decode_array(const void *buf, int offset, int maxlen, random_graph_data_agent_data *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].init_pos_), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].idx_), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __random_graph_data_agent_data_decode_array_cleanup(random_graph_data_agent_data *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int32_t_decode_array_cleanup(&(p[element].init_pos_), 1);

        __int32_t_decode_array_cleanup(&(p[element].idx_), 1);

    }
    return 0;
}

int random_graph_data_agent_data_decode(const void *buf, int offset, int maxlen, random_graph_data_agent_data *p)
{
    int pos = 0, thislen;
    int64_t hash = __random_graph_data_agent_data_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __random_graph_data_agent_data_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int random_graph_data_agent_data_decode_cleanup(random_graph_data_agent_data *p)
{
    return __random_graph_data_agent_data_decode_array_cleanup(p, 1);
}

int __random_graph_data_agent_data_clone_array(const random_graph_data_agent_data *p, random_graph_data_agent_data *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int32_t_clone_array(&(p[element].init_pos_), &(q[element].init_pos_), 1);

        __int32_t_clone_array(&(p[element].idx_), &(q[element].idx_), 1);

    }
    return 0;
}

random_graph_data_agent_data *random_graph_data_agent_data_copy(const random_graph_data_agent_data *p)
{
    random_graph_data_agent_data *q = (random_graph_data_agent_data*) malloc(sizeof(random_graph_data_agent_data));
    __random_graph_data_agent_data_clone_array(p, q, 1);
    return q;
}

void random_graph_data_agent_data_destroy(random_graph_data_agent_data *p)
{
    __random_graph_data_agent_data_decode_array_cleanup(p, 1);
    free(p);
}

int random_graph_data_agent_data_publish(lcm_t *lc, const char *channel, const random_graph_data_agent_data *p)
{
      int max_data_size = random_graph_data_agent_data_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = random_graph_data_agent_data_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _random_graph_data_agent_data_subscription_t {
    random_graph_data_agent_data_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void random_graph_data_agent_data_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    random_graph_data_agent_data p;
    memset(&p, 0, sizeof(random_graph_data_agent_data));
    status = random_graph_data_agent_data_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding random_graph_data_agent_data!!!\n", status);
        return;
    }

    random_graph_data_agent_data_subscription_t *h = (random_graph_data_agent_data_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    random_graph_data_agent_data_decode_cleanup (&p);
}

random_graph_data_agent_data_subscription_t* random_graph_data_agent_data_subscribe (lcm_t *lcm,
                    const char *channel,
                    random_graph_data_agent_data_handler_t f, void *userdata)
{
    random_graph_data_agent_data_subscription_t *n = (random_graph_data_agent_data_subscription_t*)
                       malloc(sizeof(random_graph_data_agent_data_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 random_graph_data_agent_data_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg random_graph_data_agent_data LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int random_graph_data_agent_data_subscription_set_queue_capacity (random_graph_data_agent_data_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int random_graph_data_agent_data_unsubscribe(lcm_t *lcm, random_graph_data_agent_data_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe random_graph_data_agent_data_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

