#ifndef __LINKVERSION_H
#define __LINKVERSION_H

/* Protocol Version */
#define VER_MAJOR                      (1) /* Major */
#define VER_MINOR                      (8) /* Minor */
#define VER_FIXNUM                     (0) /* Fix Number */

#define VER_MAJOR_Pos                  (12)
#define VER_MAJOR_Msk                  (0xF000)
#define VER_MINOR_Pos                  (8)
#define VER_MINOR_Msk                  (0x0F00)
#define VER_FIXNUM_Pos                 (0)
#define VER_FIXNUM_Msk                 (0x00FF)

#define MAKE_VERSION(a, b, c)          (((a << VER_MAJOR_Pos) & VER_MAJOR_Msk) | \
                                        ((b << VER_MINOR_Pos) & VER_MINOR_Msk) | \
                                        ((c << VER_FIXNUM_Pos) & VER_FIXNUM_Msk))

#define PROTOCOL_VERSION               MAKE_VERSION(VER_MAJOR, VER_MINOR, VER_FIXNUM)
#define KYLINK_MODULE_NAME             "kyLink@kyChu V"#VER_MAJOR"."#VER_MINOR"."#VER_FIXNUM

#endif /* __LINKVERSION_H */

/******************** kyChu<kyChu@qq.com> **** END OF FILE ********************/
