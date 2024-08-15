#include "pb.h"

const pb_msgdesc_t *decode_unionmessage_type(pb_istream_t *stream);

bool decode_unionmessage_contents(pb_istream_t *stream,
                                  const pb_msgdesc_t *messagetype,
                                  void *dest_struct);
