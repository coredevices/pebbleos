# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_base64
  SOURCES
    src/fw/util/base64.c
)

pbl_clar_test(test_shared_circular_buffer
  SOURCES
    src/fw/util/shared_circular_buffer.c
)

pbl_clar_test(test_time
  SOURCES
    src/fw/util/time/time.c
    src/fw/util/time/mktime.c
    tests/fakes/fake_rtc.c
)

pbl_clar_test(test_dict
  SOURCES
    src/fw/util/dict.c
)

pbl_clar_test(test_buffer
  SOURCES
    src/fw/util/buffer.c
)

pbl_clar_test(test_lru_cache
  SOURCES
    src/fw/util/lru_cache.c
)

pbl_clar_test(test_mbuf
  SOURCES
    src/fw/util/mbuf.c
    src/fw/util/mbuf_iterator.c
)

pbl_clar_test(test_rand
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
)

pbl_clar_test(test_stats
  SOURCES
    src/fw/util/stats.c
)

pbl_clar_test(test_legacy_checksum
  SOURCES
    src/fw/util/legacy_checksum.c
)

pbl_clar_test(test_generic_attribute
  SOURCES
    src/fw/util/rand/rand.c
    src/fw/util/generic_attribute.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
)

pbl_clar_test(test_hdlc
  SOURCES
    src/fw/util/hdlc.c
)

pbl_clar_test(test_stringlist
  SOURCES
    src/fw/util/stringlist.c
)

pbl_clar_test(test_pstring
  SOURCES
    src/fw/util/pstring.c
)

pbl_clar_test(test_ihex
  SOURCES
    src/fw/util/ihex.c
)

pbl_clar_test(test_sle
  SOURCES
    src/fw/util/sle.c
)
