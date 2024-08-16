#include "pb.h"

bool encode_unionmessage(pb_ostream_t *stream,
                         const pb_msgdesc_t *messagetype,
                         void *message);

pb_ostream_t pb_ostream_from_socket(int fd);

pb_istream_t pb_istream_from_socket(int fd);

const pb_msgdesc_t *decode_unionmessage_type(pb_istream_t *stream);

bool decode_unionmessage_contents(pb_istream_t *stream,
                                  const pb_msgdesc_t *messagetype,
                                  void *dest_struct);
