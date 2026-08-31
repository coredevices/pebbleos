/*-
 * Copyright (c) 2000, 2001 Citrus Project,
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pbl/services/i18n/i18n.h"
#include "pbl/services/i18n/mo.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "resource/resource.h"
#include "shell/normal/language_ui.h"
#include "shell/prefs.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/list.h"

PBL_LOG_MODULE_DEFINE(service_i18n, CONFIG_SERVICE_I18N_LOG_LEVEL);

//////////////////////////////////////////////////////
// See mo.h for a description of the MO file format //
//////////////////////////////////////////////////////

typedef struct {
  uint32_t hash;
  const char *string;
  const void *owner;
} StringLookupInfo;

static struct DomainBinding {
  uint32_t resource_id;
  ResourceCallbackHandle watch_handle;
  bool need_reload;
  ResourceVersion version;
  MoHandle mohandle;
  I18nString *strings_list;
  char iso_locale[ISO_LOCALE_LENGTH];
  char lang_name[LOCALE_NAME_LENGTH];
  uint16_t lang_version;
} s_system_domain;

static void prv_list_flush(void);

///////////////////////////////////////////////////
// MO File Hash Table

uint32_t prv_gettext_hash(const char *str) {
  const uint8_t *p;
  uint32_t hash = 0, tmp;

  for (p = (const uint8_t *)str; *p; p++) {
    hash <<= 4;
    hash += *p;
    tmp = hash & 0xF0000000;
    if (tmp != 0) {
      hash ^= tmp;
      hash ^= tmp >> 24;
    }
  }

  return hash;
}

static uint32_t prv_collision_step(uint32_t hashval, uint32_t hashsize) {
  return (hashval % (hashsize - 2)) + 1;
}

static uint32_t prv_next_index(uint32_t curidx, uint32_t hashsize, uint32_t step) {
  return curidx + step - (curidx >= hashsize - step ? hashsize : 0);
}

//! Lookup a translated string.
//! @param msgid The message id (original string) to look up.
//! @param db The domain binding containing the translations.
//! @param[out] rlen Can be NULL. If non-null will be populated with the length of the translated
//!                  string.
//! @param[out] rstring Can be NULL. If non-null this buffer will be populated with the translated
//!                     string. This buffer will be null-terminated.
//! @param rstring_len The length of the rstring buffer.
static void prv_lookup(const char *msgid, struct DomainBinding *db,
                       size_t *rlen, char *rstring, size_t rstring_len) {
  MoHandle *mohandle = &db->mohandle;
  *rlen = 0;

  if (mohandle->mo.hdr.mo_hsize <= 2 || mohandle->mo.mo_htable == NULL) {
    return;
  }

  uint32_t hashval = prv_gettext_hash(msgid);
  uint32_t step = prv_collision_step(hashval, mohandle->mo.hdr.mo_hsize);
  uint32_t idx = hashval % mohandle->mo.hdr.mo_hsize;
  size_t len = strlen(msgid);
  while (1) {
    uint32_t strno = mohandle->mo.mo_htable[idx];
    if (strno-- == 0) {
      /* unexpected miss */
      return;
    }
    MoEntry oentry;
    if (resource_load_byte_range_system(0, db->resource_id, mohandle->mo.hdr.mo_otable
            + sizeof(MoEntry) * strno, (uint8_t *)&oentry, sizeof(MoEntry)) != sizeof(MoEntry)) {
      return;
    }
    if (len == oentry.len) {
      // Length of original matches, compare the contents
      char key[oentry.len + 1];
      if (resource_load_byte_range_system(0, db->resource_id, oentry.off, (uint8_t *)key,
            oentry.len) != oentry.len) {
        return;
      }
      key[oentry.len] = '\0';

      if (!strcmp(msgid, key)) {
        // Contents of original string matches, get the translated string
        MoEntry tentry;
        if (resource_load_byte_range_system(0, db->resource_id, mohandle->mo.hdr.mo_ttable
            + sizeof(MoEntry) * strno, (uint8_t *)&tentry, sizeof(MoEntry)) != sizeof(MoEntry)) {
          return;
        }
        if (rstring) { // If we want the translated string, copy it out.
          // Make sure we don't read out more than the length of the buffer we're reading into.
          // Leave space for the null-terminator as well.
          const size_t read_length = MIN(tentry.len, rstring_len - 1);

          if (resource_load_byte_range_system(0, db->resource_id, tentry.off,
                                              (uint8_t *)rstring, read_length) != read_length) {
            return;
          }

          rstring[read_length] = '\0';
        }
        if (rlen) { // If we want the translated string length, copy it out.
          *rlen = tentry.len;
        }
        return;
      }
    }
    idx = prv_next_index(idx, mohandle->mo.hdr.mo_hsize, step);
  }
}

///////////////////////////////////////////////////
// MO File Mapping & Lookup

static bool prv_get_property(const char *header, const char *name, char *buffer, size_t size) {
  // Isolate the language name
  char *str = strstr(header, name);
  if (str == NULL) { // strstr failed
    return false;
  }
  str += strlen(name);

  char *end = strchr(str, '\n');
  unsigned int length = end - str;
  if (end == NULL || length > size) { // strchr failed
    return false;
  }

  memcpy(buffer, str, length);
  buffer[length] = '\0';
  return true;
}

static bool prv_get_metadata(struct DomainBinding *db) {
  const size_t HEADER_BUFFER_SIZE = 400;
  // malloc a comfortable amount of RAM to save the header in
  char *header = kernel_malloc_check(HEADER_BUFFER_SIZE);
  size_t header_len = 0;
  bool success = false;

  // all metadata is in the "" header entry
  prv_lookup("", db, &header_len, header, HEADER_BUFFER_SIZE);
  if (!header_len) {
    PBL_LOG_WRN("Could not find header in language pack");
    goto cleanup;
  }

  // Isolate the language substring
  if (!prv_get_property(header, "Language: ", db->iso_locale, ISO_LOCALE_LENGTH)) {
    PBL_LOG_WRN("Could not parse a language from language pack");
    goto cleanup;
  }

  // Isolate the language name
  if (!prv_get_property(header, "Name: ", db->lang_name, LOCALE_NAME_LENGTH)) {
    strcpy(db->lang_name, "Unknown");
  }

  // Isolate the version value
  char version_str[10] = {0};
  if (!prv_get_property(header, "Project-Id-Version: ", version_str, 10)) {
    PBL_LOG_WRN("Could not parse a version from language pack");
    goto cleanup;
  }
  char *version_end;
  db->lang_version = strtol(version_str, &version_end, 0);
  if (version_end == version_str) {
    PBL_LOG_WRN("Could not parse a version from language pack");
    goto cleanup;
  }

  success = true;
  PBL_LOG_DBG("language: %s, version %d", db->iso_locale, db->lang_version);

cleanup:
  kernel_free(header);
  return (success);
}

static int prv_unmapit(struct DomainBinding *db) {
  MoHandle *mohandle = &db->mohandle;

  kernel_free(mohandle->mo.mo_htable);
  mohandle->mo.mo_htable = NULL;
  mohandle->mo = (Mo){};
  strcpy(db->iso_locale, "en_US");
  strcpy(db->lang_name, "English");
  db->lang_version = 1;

  return 0;
}

static bool prv_mapit(const uint32_t resource_id, struct DomainBinding *db) {
  // If the resource is changed on disk, our resource_watch callback will set need_reload
  if (!db->need_reload) {
    return (db->version.crc != 0);
  }

  PBL_LOG_DBG("New language detected!");
  db->need_reload = false;

  /* save version */
  db->version = resource_get_version(SYSTEM_APP, resource_id);
  prv_list_flush();
  prv_unmapit(db);

  unsigned int size;
  if ((size = resource_size(SYSTEM_APP, resource_id)) < sizeof(MoHeader)) {
    goto fail;
  }

  if (!resource_is_valid(SYSTEM_APP, resource_id)) {
    goto fail;
  }

  MoHandle *mohandle = &db->mohandle;
  if (resource_load_byte_range_system(SYSTEM_APP, resource_id, 0,
      (uint8_t *)&mohandle->mo.hdr, sizeof(MoHeader)) == 0) {
    goto fail;
  }
  if (mohandle->mo.hdr.mo_magic != MO_MAGIC) {
    goto fail;
  }

  mohandle->len = size;
    /* validate htable */
  if (mohandle->mo.hdr.mo_hsize < 2) {
    goto fail;
  }

  size_t htable_size = sizeof(uint32_t) * mohandle->mo.hdr.mo_hsize;
  uint32_t *htable = kernel_malloc_check(htable_size);
  mohandle->mo.mo_htable = htable;
  if (resource_load_byte_range_system(SYSTEM_APP, resource_id, mohandle->mo.hdr.mo_hoffset,
      (uint8_t *)htable, htable_size) == 0) {
    prv_unmapit(db);
    goto fail;
  }

  for (unsigned int i = 0; i < mohandle->mo.hdr.mo_hsize; ++i) {
    if (htable[i] > mohandle->mo.hdr.mo_nstring) {
      /* illegal string number */
      prv_unmapit(db);
      goto fail;
    }
  }

  if (!prv_get_metadata(db)) {
      prv_unmapit(db);
      goto fail;
  }

  return true;

fail:
  return false;
}

///////////////////////////////////////////////////
// Strings List Manipulation

void prv_list_flush(void) {
  ListNode *cur = (ListNode *)s_system_domain.strings_list;
  while (cur) {
    ListNode *next = list_get_next(cur);
    kernel_free(cur);
    cur = next;
  }
  s_system_domain.strings_list = NULL;
}

static bool prv_list_string_filter_callback(ListNode *found_node, void *data) {
  I18nString *i18n_string = (I18nString *)found_node;
  StringLookupInfo *lookup_info = data;
  if (i18n_string->original_hash == lookup_info->hash &&
      lookup_info->owner == i18n_string->owner &&
      strcmp(i18n_string->original_string, lookup_info->string) == 0) {
    return true;
  } else {
    return false;
  }
}

static bool prv_list_owner_filter_callback(ListNode *found_node, void *owner) {
  I18nString *i18n_string = (I18nString *)found_node;
  if (i18n_string->owner == owner) {
    return true;
  } else {
    return false;
  }
}

// Not static because we call this from unit test code
I18nString *prv_list_find_string(const char *string, const void *owner) {
  StringLookupInfo lookup_info = {
    .string = string,
    .hash = prv_gettext_hash(string),
    .owner = owner
  };
  return (I18nString *)list_find((ListNode *)s_system_domain.strings_list,
      prv_list_string_filter_callback, (void *)&lookup_info);
}


static const char *prv_list_add_string(const char *original_string, const char *translated_string,
                                       const void *owner) {
  uint32_t translated_len = strlen(translated_string);

  // Allocate enough space to hold the original and translated strings. The translated string
  // is stored at i18n_string->translated and the original string immediately after that.
  I18nString *i18n_string = kernel_malloc_check(sizeof(I18nString) + translated_len + 1
              + strlen(original_string) + 1);

  list_init(&i18n_string->node);
  i18n_string->owner = owner;

  strcpy(i18n_string->translated_string, translated_string);

  i18n_string->original_hash = prv_gettext_hash(original_string);
  // Store the original string immediately after the translated one in memory. 
  i18n_string->original_string = &i18n_string->translated_string[translated_len + 1];
  strcpy(i18n_string->original_string, original_string);

  I18nString **strings_list = &s_system_domain.strings_list;
  *strings_list = (I18nString *)list_prepend((ListNode *)*strings_list, &i18n_string->node);

  if (translated_len > 0) {
    return (i18n_string->translated_string);
  } else {
    return original_string;
  }
}

static void prv_list_remove_string(I18nString *i18n_string) {
  list_remove(&i18n_string->node, (ListNode **)&s_system_domain.strings_list, NULL);
  kernel_free(i18n_string);
}

static bool prv_check_domain(struct DomainBinding *db) {
  return (prv_mapit(s_system_domain.resource_id, db));
}

static const char *prv_message_from_msgid(const char *msgid) {
  // If a string wasn't found, we want to return the original string.
  // However, if we have a context, this string needs to not show the context.
  // So we just find EOT and if it's present return the next character.
  const char *message = strchr(msgid, '\4');
  if (message == NULL) {
    // No context, the whole string is the message.
    return msgid;
  }
  // strchr gets the address of that character. We want to skip the EOT, so +1.
  return message + 1;
}

///////////////////////////////////////////////////
// i18n API

// NOTE: Currently, we don't do reference counting, so bad things will happen if the caller
// calls i18n_get() on the same string more than once and assumes that any of those return
// pointers will still be valid after i18n_free() is called on one of them.
const char *i18n_get(const char *msgid, const void *owner) {
  PBL_ASSERTN(owner);
  if (msgid == NULL || msgid[0] == 0) {
    goto fail;
  }

  struct DomainBinding *db = &s_system_domain;
  if (!prv_check_domain(db)) {
    goto fail;
  }
  // See if this original has been cached.
  I18nString *i18n_string = prv_list_find_string(msgid, owner);
  if (i18n_string) {
    if (i18n_string->translated_string[0]) {
      return i18n_string->translated_string;
    } else {
      // No translation exists for this string, return original
      goto fail;
    }
  }

  // Lookup the translation from the language pack and add it to our cache
  char translated[200];
  size_t len = 0;
  prv_lookup(msgid, db, &len, translated, sizeof(translated));
  if (len >= sizeof(translated)) {
    PBL_LOG_WRN("Truncated string: <%s>", msgid);
  }

  if (len) {
    return prv_list_add_string(msgid, translated, owner);
  } else {
    // Add to cache as an untranslatable string so we don't waste time looking for it again.
    prv_list_add_string(msgid, (const char *)"", owner);
  }

fail:
  // String not found or an error occurred.
  return prv_message_from_msgid(msgid);
}

void i18n_get_with_buffer(const char *msgid, char *buffer, size_t length) {
  if (length == 0) {
    // Nothing fits, and buffer[length - 1] below would wrap to an OOB write.
    return;
  }
  if (msgid == NULL || msgid[0] == 0) {
    goto fail;
  }

  struct DomainBinding *db = &s_system_domain;
  if (!prv_check_domain(db)) {
    goto fail;
  }

  size_t len = 0;
  prv_lookup(msgid, db, &len, buffer, length);
  if (len >= length) {
    PBL_LOG_WRN("Truncated string: <%s>", msgid);
  }

  if (len) {
    // buffer has been written, return
    return;
  }

fail:
  msgid = prv_message_from_msgid(msgid);
  strncpy(buffer, msgid, length);
  buffer[length - 1] = '\0';
}

size_t i18n_get_length(const char *msgid) {
  if (msgid == NULL || msgid[0] == 0) {
    return 0;
  }

  struct DomainBinding *db = &s_system_domain;
  if (!prv_check_domain(db)) {
    goto fail;
  }

  size_t len = 0;
  prv_lookup(msgid, db, &len, NULL, 0);
  if (len) { // String was found
    return len;
  }

fail:
  // String not found, or error occurred
  msgid = prv_message_from_msgid(msgid);
  return strlen(msgid);
}

void i18n_free(const char *original, const void *owner) {
  PBL_ASSERTN(owner);
  I18nString *i18n_string = prv_list_find_string(original, owner);
  if (i18n_string) {
    prv_list_remove_string(i18n_string);
  }
}

void i18n_free_all(const void *owner) {
  I18nString *cur_string = (I18nString *)list_find((ListNode *)s_system_domain.strings_list,
      prv_list_owner_filter_callback, (void*)owner);
  while (cur_string) {
    I18nString *next_string = (I18nString *)list_find_next(&cur_string->node,
        prv_list_owner_filter_callback, false, (void*)owner);
    prv_list_remove_string(cur_string);
    cur_string = next_string;
  }
}

static void prv_resource_changed_handler(void *data) {
  struct DomainBinding *db = (struct DomainBinding *)data;
  // Mark as invalid
  PBL_LOG_DBG("lang resource file reloading");
  shell_prefs_set_language_english(false);
  db->need_reload = true;

  if (resource_is_valid(SYSTEM_APP, db->resource_id)) {
    language_ui_display_changed(db->lang_name);
  }
}

static void prv_resource_changed_callback(void *data) {
  // We want to not actually handle the reload here, because the PFS lock is still held here.
  // So instead we throw in the reload as an event callback.
  PBL_LOG_DBG("lang resource file was modified");
  launcher_task_add_callback(prv_resource_changed_handler, data);
}

static void prv_unset(void) {
  s_system_domain.need_reload = false;
  prv_list_flush();
  prv_unmapit(&s_system_domain);
}

void i18n_set_resource(uint32_t resource_id) {
  // Remove prior watch, if any
  // Warning: you better be sure we're not calling from the resource changed callback.
  if (s_system_domain.watch_handle) {
    resource_unwatch(s_system_domain.watch_handle);
  }

  s_system_domain.resource_id = resource_id;
  s_system_domain.watch_handle = resource_watch(SYSTEM_APP, resource_id,
                                                prv_resource_changed_callback, &s_system_domain);

  if (shell_prefs_get_language_english()) {
    prv_unset();
    return;
  }

  s_system_domain.need_reload = true;

  // try mapping it right away
  prv_mapit(resource_id, &s_system_domain);
}

char *i18n_get_locale(void) {
  return (s_system_domain.iso_locale);
}

uint16_t i18n_get_version(void) {
  return (s_system_domain.lang_version);
}

char *i18n_get_lang_name(void) {
  return (s_system_domain.lang_name);
}

void i18n_enable(bool enable) {
  if (enable) {
    s_system_domain.need_reload = true;
    prv_mapit(s_system_domain.resource_id, &s_system_domain);
  } else {
    prv_unset();
  }
}

void command_i18n_resource(const char *arg) {
  uint32_t resource_id = atoi(arg);
  i18n_set_resource(resource_id);
}

