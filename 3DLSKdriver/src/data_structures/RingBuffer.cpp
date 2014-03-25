//http://www.osix.net/modules/article/?id=464

//const unsigned long BUFFER_SIZE = 10;


template<typename T, unsigned int bufferSize>
class RingBuffer {
private:
    T buffer[bufferSize];
    unsigned int current_element;
public:
    RingBuffer() : current_element(0) {
    }

    RingBuffer(const RingBuffer& old_ring_buf) {
        memcpy(buffer, old_ring_buf.buffer, bufferSize*sizeof(T));
        current_element = old_ring_buf.current_element;
    }

    RingBuffer operator = (const RingBuffer& old_ring_buf) {
        memcpy(buffer, old_ring_buf.buffer, bufferSize*sizeof(T));
        current_element = old_ring_buf.current_element;
    }

    ~RingBuffer() { }

    void append(T value) {
        if(current_element >= bufferSize) {
            current_element = 0;
        }

        buffer[current_element] = value;

        ++current_element;
    }

    T get() {
        if(current_element >= bufferSize) {
            current_element = 0;
        }

        ++current_element;
        return( buffer[(current_element-1)] );
    }

    unsigned int current() {
        return (current_element);
    }

    unsigned int size(){
	return bufferSize;
    }
};
