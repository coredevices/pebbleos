#pragma once
#ifndef RECOVERY_FW
void init_memfault_chunk_collection(void);
#else
static inline void init_memfault_chunk_collection(void) {}
#endif