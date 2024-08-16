#include "pb.h"

static bool write_callback(pb_ostream_t *stream,
                           const uint8_t *buf,
                           size_t count);

static bool read_callback(pb_istream_t *stream, uint8_t *buf, size_t count);

pb_ostream_t pb_ostream_from_socket(int fd);

pb_istream_t pb_istream_from_socket(int fd);

const pb_msgdesc_t *decode_unionmessage_type(pb_istream_t *stream);

bool decode_unionmessage_contents(pb_istream_t *stream,
                                  const pb_msgdesc_t *messagetype,
                                  void *dest_struct);
