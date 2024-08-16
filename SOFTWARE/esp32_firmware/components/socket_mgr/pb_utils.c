// https://github.com/nanopb/nanopb/tree/e34dfae6f1aeb87cbe1c05a8fb96f5b9cb1034f6/examples/using_union_messages

#include "pb_utils.h"

#include "messages.pb.h"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"

bool encode_unionmessage(pb_ostream_t *stream,
                         const pb_msgdesc_t *messagetype,
                         void *message)
{
    pb_field_iter_t iter;

    if (!pb_field_iter_begin(&iter, UdpPacket_fields, message))
        return false;

    do {
        if (iter.submsg_desc == messagetype) {
            /* This is our field, encode the message using it. */
            if (!pb_encode_tag_for_field(stream, &iter))
                return false;

            return pb_encode_submessage(stream, messagetype, message);
        }
    } while (pb_field_iter_next(&iter));

    /* Didn't find the field for messagetype */
    return false;
}

const pb_msgdesc_t *decode_unionmessage_type(pb_istream_t *stream)
{
    pb_wire_type_t wire_type;
    uint32_t tag;
    bool eof;

    while (pb_decode_tag(stream, &wire_type, &tag, &eof)) {
        if (wire_type == PB_WT_STRING) {
            pb_field_iter_t iter;
            if (pb_field_iter_begin(&iter, UdpPacket_fields, NULL) &&
                pb_field_iter_find(&iter, tag)) {
                /* Found our field. */
                return iter.submsg_desc;
            }
        }

        /* Wasn't our field.. */
        pb_skip_field(stream, wire_type);
    }

    return NULL;
}

bool decode_unionmessage_contents(pb_istream_t *stream,
                                  const pb_msgdesc_t *messagetype,
                                  void *dest_struct)
{
    pb_istream_t substream;
    bool status;
    if (!pb_make_string_substream(stream, &substream))
        return false;

    status = pb_decode(&substream, messagetype, dest_struct);
    pb_close_string_substream(stream, &substream);
    return status;
}
