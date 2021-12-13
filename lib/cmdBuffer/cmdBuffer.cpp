/*
    Member functions for CmdBuffer class.
*/

#include <ringbuffer.h>
#include <cmdBuffer.h>

// Constructor
CmdBuffer::CmdBuffer(size_t size)
            : numCmds(0)
        {
            // Buffer that will be used to store commands received over BT
            rbSerBuf = new ringbuffer<uint8_t>(size);
        } // CmdBuffer()

size_t CmdBuffer::readToEnd(uint8_t * dest)
        {
            size_t n = rbSerBuf->getOccupied();
            if (n > 0) 
            {
                rbSerBuf->read(dest, n);
            }
            return n;
        }

size_t CmdBuffer::write(uint8_t * data, size_t n)
        {
            if(rbSerBuf->getFree() >= n)
            {
                rbSerBuf->write(data, n);
                return n;
            } 
            else 
            {
                return 0;
            }
        }