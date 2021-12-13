#ifndef CMD_BUFFER_H
#define CMD_BUFFER_H

#include <ringbuffer.h>

class CmdBuffer {
    public:
    
        CmdBuffer(size_t size);
        int getNumCmds(void);
        size_t readToEnd(uint8_t * dest);
        size_t write(uint8_t * data, size_t n);
        size_t getOccupied(void) { return rbSerBuf->getOccupied(); }
        size_t getFree(void){ return rbSerBuf->getFree(); }

                    
    private:
        int numCmds;
        ringbuffer<uint8_t>* rbSerBuf;

}; // class cmdBuffer

#endif // CMD_BUFFER_H