/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.0-dev */

#ifndef PB_COMMAND_PB_H_INCLUDED
#define PB_COMMAND_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _Command {
    int32_t type;
    bool has_id;
    int32_t id;
    bool has_header;
    int32_t header;
    pb_callback_t debug_info;
/* @@protoc_insertion_point(struct:Command) */
} Command;


/* Initializer values for message structs */
#define Command_init_default                     {0, false, 0, false, 0, {{NULL}, NULL}}
#define Command_init_zero                        {0, false, 0, false, 0, {{NULL}, NULL}}

/* Field tags (for use in manual encoding/decoding) */
#define Command_type_tag                         1
#define Command_id_tag                           2
#define Command_header_tag                       3
#define Command_debug_info_tag                   4

/* Struct field encoding specification for nanopb */
#define Command_FIELDLIST(X, a) \
X(a, STATIC, REQUIRED, INT32, type, 1) \
X(a, STATIC, OPTIONAL, INT32, id, 2) \
X(a, STATIC, OPTIONAL, INT32, header, 3) \
X(a, CALLBACK, REPEATED, INT32, debug_info, 4)
#define Command_CALLBACK pb_default_field_callback
#define Command_DEFAULT NULL

extern const pb_msgdesc_t Command_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Command_fields &Command_msg

/* Maximum encoded size of messages (where known) */
/* Command_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
