/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/voice/transcription.h"

#include "system/passert.h"

// Check that there are no null terminators or special characters in the word
static bool prv_validate_word(utf8_t *str, uint16_t size) {
  for (uint16_t i = 0; i < size; i++) {
    if ((str[i] < (utf8_t)' ') && (str[i] != '\x08')) {
      return false;
    }
  }
  return true;
}

bool transcription_validate(const Transcription *transcription, size_t size) {
  if (!transcription ||
      (size <= sizeof(Transcription)) ||
      (transcription->type != TranscriptionTypeSentenceList)) {
    return false;
  }

  uint8_t *end = (uint8_t *)transcription + size;

  uint8_t *cursor = NULL;
  const TranscriptionSentence *sentence = transcription->sentences;

  for (size_t i = 0; i < transcription->sentence_count; i++) {
    cursor = (uint8_t *) sentence->words;

    // Check that sentence header fits into buffer and length is valid
    if ((cursor >= end) || (sentence->word_count == 0)) {
      return false;
    }

    for (size_t j = 0; j < sentence->word_count; j++) {
      TranscriptionWord *word = (TranscriptionWord *) cursor;
      cursor = (uint8_t *) word->data;

      // Check that word header fits into buffer and length is valid
      if ((cursor >= end) || (word->length == 0) || (word->data + word->length > end) ||
          !prv_validate_word(word->data, word->length)) {
        return false;
      }

      cursor += word->length;
    }
    sentence = (TranscriptionSentence *)cursor;
  }

  return (cursor == end);
}

void *transcription_iterate_sentences(const TranscriptionSentence *sentence, size_t count,
    TranscriptionSentenceIterateCb handle_sentence, void *data) {

  for (size_t i = 0; i < count; i++) {
    if (handle_sentence && !handle_sentence(sentence, data)) {
      // end iteration if callback returns false
      break;
    }

    sentence = transcription_iterate_words(sentence->words, sentence->word_count, NULL, NULL);
  }
  return (void *)sentence;
}

void *transcription_iterate_words(const TranscriptionWord *words, size_t count,
    TranscriptionWordIterateCb handle_word, void *data) {

  uint8_t *cursor = (uint8_t *)words;
  for (size_t i = 0; i < count; i++) {
    TranscriptionWord *word = (TranscriptionWord *) cursor;

    if (handle_word && !handle_word(word, data)) {
      // end iteration if callback returns false
      break;
    }

    cursor += sizeof(TranscriptionWord) + word->length;
  }

  return cursor;
}
