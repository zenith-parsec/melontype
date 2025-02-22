#ifndef __font_h__
#define __font_h__

// Define digit bitmaps (5x8 for each digit, stored as 5 bytes per digit)
const uint8_t digitBitmaps[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x62, 0x51, 0x49, 0x49, 0x46}, // 2
    {0x22, 0x49, 0x49, 0x49, 0x36}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x2F, 0x49, 0x49, 0x49, 0x31}, // 5
    {0x3E, 0x49, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}  // 9
};

// back to my roots: i asked chatgpt to write this.
// I had already written the same thing earlier today,
// but it was slightly uglier. surprisingly, the only real
// difference between the two functions was that it was much
// cleaner, with fewer random weirdnesses
//
// Function to get column data based on number and phase
// columnData should be uint8_t cd[8]; and passed as &cd[0]
// returns array of 0 and 1 for respective LED off and on.
void getColumnData(uint16_t number, float phase, bool reverse, uint8_t* columnData) {
    char numStr[10];
    snprintf(numStr, sizeof(numStr), "%5u", number); // Convert number to string
    uint8_t numDigits = strlen(numStr);

    const uint8_t charSpacing = 1;  // Space between digits
    const uint8_t digitWidth = 5;   // Width of each digit

    uint8_t totalCols = numDigits * (digitWidth + charSpacing) - charSpacing;
    uint8_t colIndex = (uint8_t)(phase * totalCols) % totalCols;

    if (reverse) {
        colIndex = totalCols - 1 - colIndex;
    }

    // Determine which digit and column within that digit
    uint8_t digitIdx = colIndex / (digitWidth + charSpacing);
    uint8_t colWithinDigit = colIndex % (digitWidth + charSpacing);

    if (colWithinDigit >= digitWidth) {
        memset(columnData, 0, 7); // Return empty column for spacing
        return;
    }

    uint8_t digit = numStr[digitIdx] - '0'; // Convert character to digit
    uint8_t columnByte = digitBitmaps[digit][colWithinDigit];

    // Copy only the lower 7 bits (high bit is always 0)
    for (uint8_t i = 0; i < 7; i++) {
        columnData[i] = (columnByte >> i) & 1;
    }
}

#endif