#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file circular_buffer_index.h
 * @brief Ring buffer index tracking and chronological slot address mapping.
 *
 * Encapsulates head write pointer, stored item count, capacity limits,
 * and chronological order translation for fixed-size circular storage.
 */
class CircularBufferIndex {
public:
    explicit CircularBufferIndex(uint8_t capacity)
        : _capacity(capacity), _head(0), _count(0) {}

    CircularBufferIndex(uint8_t capacity, uint8_t initialHead, uint8_t initialCount)
        : _capacity(capacity), _head(initialHead), _count(initialCount) {
        sanitize();
    }

    uint8_t capacity() const { return _capacity; }
    uint8_t count() const { return _count; }
    uint8_t head() const { return _head; }
    bool isEmpty() const { return _count == 0; }
    bool isFull() const { return _count >= _capacity; }

    /**
     * @brief Sanitizes head and count to guarantee they are within valid capacity limits.
     */
    void sanitize() {
        if (_count > _capacity) {
            _count = _capacity;
        }
        if (_head >= _capacity) {
            _head = 0;
        }
    }

    /**
     * @brief Allocates the current write slot and advances head and count.
     * @return The slot index where the new item should be stored.
     */
    uint8_t advanceWrite() {
        uint8_t slot = _head;
        _head = (_head + 1) % _capacity;
        if (_count < _capacity) {
            _count++;
        }
        return slot;
    }

    /**
     * @brief Translates a chronological index (0 = oldest, count - 1 = newest) into the physical slot.
     * @param chronologicalIndex Index from 0 to count - 1.
     * @return Physical slot index in [0, capacity - 1].
     */
    uint8_t getChronologicalSlot(uint8_t chronologicalIndex) const {
        if (_count == 0) return 0;
        uint8_t startSlot = (_count == _capacity) ? _head : 0;
        return (startSlot + chronologicalIndex) % _capacity;
    }

    /**
     * @brief Resets head and count to 0.
     */
    void reset() {
        _head = 0;
        _count = 0;
    }

    /**
     * @brief Restores state from external values (e.g. read from non-volatile storage).
     */
    void restore(uint8_t head, uint8_t count) {
        _head = head;
        _count = count;
        sanitize();
    }

private:
    uint8_t _capacity;
    uint8_t _head;
    uint8_t _count;
};
