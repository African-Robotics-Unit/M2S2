class frameBuffer():

    # %---------------------------------------------------------------------------------------------------------
    # Class Parameters
    # %---------------------------------------------------------------------------------------------------------
    capacity = None
    frameSize = None
    tail = None
    head = None
    bytesStored = None
    buf = None

    # %---------------------------------------------------------------------------------------------------------
    # Class Functions
    # %---------------------------------------------------------------------------------------------------------    
    # Class constructor
    def __init__(self,frameSize,capMultiplier=3) -> None:
        """Initializes a circular buffer to store frames of data.

    This class is designed to buffer incoming radar data
    so that frames of radar data can be put together and 
    extracted.

    :param int frameSize: Size of the radar frame in bytes.
    :param int capMultiplier: Multiplies frameSize to get total buffer length. (default = 3)
    :param int chunkDivisor: Sets the size of data returned from getChunk method. Must divide into frame size. (default = 8)
    :raises AnyError: prints warning if chunkDivisor does not divide into frameSize
    """
        self.capacity = capMultiplier*frameSize
        self.frameSize = frameSize
        self.tail = -1
        self.head = 0
        self.bytesStored = 0  # number of bytes stored from current frame
        self.buf = [b'\x00']*(self.capacity) # initialize list with zero bytes
    
    # Primary Functions
    # %---------------------------------------------------------------------------------------------------------
    def Enqueue(self,item):
        """Adds an item to buffer

    This fucntion adds an item to the back of the queue.
    It automatically will go back to start of buffer if buffer is full
    and will automatically overwrite old data. (Becareful of overwriting 
    data that has not been exported out of buffer yet)

    :param bytes item: Single byte of radar data i.e. b"\\xff". 
    """
        self.tail = (self.tail + 1) % self.capacity # increase tail of buffer and wrap to start if capacity is reached
        self.buf[self.tail] = item # set value at tail index to item
        self.bytesStored = self.bytesStored + 1 # increase the number of bytes stored

    # %---------------------------------------------------------------------------------------------------------
    def getFrame(self):
        """Removes mutliple items from buffer

    This fucntion works similarly to Dequeue but will return one frame of data.
    It will automatically adjust head position and bytesStored. If frame of data is not ready (if there aren't 
    enough new bytes stored) it will return None. You can use this to check if function was successful. 

    :returns: frame (length = frameSize)
    :rtype: [bytes] or None
    """
        frame = None
        if self.bytesStored >= self.frameSize: # check if frame is ready/large enough
            frame = self.buf[self.head:self.head+self.frameSize]
            self.head = (self.head+self.frameSize) % self.capacity
            self.bytesStored -= self.frameSize
        return frame     

    # %---------------------------------------------------------------------------------------------------------
    # END OF CLASS
    # %---------------------------------------------------------------------------------------------------------