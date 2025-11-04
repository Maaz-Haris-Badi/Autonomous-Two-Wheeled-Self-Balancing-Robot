#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

// Block map: each byte represents one block
// 0 = free, 1 = allocated
static uint8_t block_map[BLOCK_COUNT];

void heap_init(void) {
    // Initialize all blocks as free (0)
    memset(block_map, 0, BLOCK_COUNT);
}

void* heap_alloc(size_t size) {
    if (size == 0) {
        return NULL;
    }
    
    // Calculate number of blocks needed (round up)
    size_t blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;
    
    if (blocks_needed > BLOCK_COUNT) {
        return NULL;  // Request too large
    }
    
    // Search for contiguous free blocks
    size_t start_block = 0;
    
    while (start_block <= BLOCK_COUNT - blocks_needed) {
        // Check if we have blocks_needed contiguous free blocks
        size_t i;
        for (i = 0; i < blocks_needed; i++) {
            if (block_map[start_block + i] != 0) {
                // This block is allocated, skip past it
                start_block += i + 1;
                break;
            }
        }
        
        // If we checked all blocks and they're all free
        if (i == blocks_needed) {
            // Mark these blocks as allocated (1)
            for (size_t j = 0; j < blocks_needed; j++) {
                block_map[start_block + j] = 1;
            }
            
            // Return pointer to the first block
            return HEAP_START_ADDR + (start_block * BLOCK_SIZE);
        }
    }
    
    // No contiguous space found
    return NULL;
}

void heap_free(void* ptr) {
    if (ptr == NULL) {
        return;
    }
    
    // Check if pointer is within heap bounds
    uint8_t* byte_ptr = (uint8_t*)ptr;
    if (byte_ptr < HEAP_START_ADDR || byte_ptr >= HEAP_START_ADDR + HEAP_SIZE) {
        return;  // Invalid pointer - outside heap
    }
    
    // Calculate offset and check alignment
    size_t offset = byte_ptr - HEAP_START_ADDR;
    
    // Check if pointer is block-aligned
    if (offset % BLOCK_SIZE != 0) {
        return;  // Invalid pointer - not aligned to block boundary
    }
    
    size_t block_index = offset / BLOCK_SIZE;
    
    // Free contiguous allocated blocks starting from block_index
    // Continue until we hit a free block (0) or end of heap
    while (block_index < BLOCK_COUNT && block_map[block_index] == 1) {
        block_map[block_index] = 0;
        block_index++;
    }
}