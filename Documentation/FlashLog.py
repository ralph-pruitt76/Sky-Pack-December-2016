# This a development model of the flash logging system.
#
# This implementation writes an ever increasing counter at the start
# of each sector as is it written, allowing for binary search of head
# and tail on startup.  The high bit of the counter remains set, until
# the sector is read, when it is cleared.  Sectors are erased ahead of
# the head as needed.  If the current head sector is read, the next
# sector has to be erased and prepared to move the head forward before
# the high bit of the current head sector is cleared.
#
# To experiment with this, run python, type "from FlashLog import *"
# and use the functions to start the flash, read, write, simulate
# interrupted erase, etc.

class Memory:
  data = [0xFF] * 16
  head = -1
  tail = -1
  counter = 0

memory = Memory()


def print_data():
  """Pretty printer to show the data with head and tail highlighted"""
  HEADC = '\033[91m'
  TAILC = '\033[92m'
  NOC = '\033[0m'

  print "HEAD: %d (counter %02X)" % (memory.head, memory.counter)
  print "TAIL: %d" % memory.tail
   
  byte = 0
  for i in range(len(memory.data)):
    if i == memory.head:
      color = HEADC
    elif i == memory.tail:
      color = TAILC
    else:
      color = ''
    print "%s%s%02X%s%s" % (color, '_' if i == memory.tail else ' ',
          memory.data[i], '*' if i == memory.head else ' ', NOC),
    byte += 1
    if byte >= 16:
      print
      byte = 0

def next(sector):
  """Return next sector, with wraparound"""
  sector += 1
  if sector >= len(memory.data):
    sector = 0
  return sector
  
def find_head():
  """Find the head (write) index"""
  begin = 0
  end = len(memory.data)
  while begin + 1 != end:
    middle = begin + (end - begin) / 2
    if (memory.data[begin] != 0xFF and \
        (memory.data[middle] & 0x7F) <= (memory.data[begin] & 0x7F)) or \
        memory.data[middle] == 0xFF:
      end = middle
    else:
      begin = middle
  memory.head = begin
  if memory.data[begin] != 0xFF:
    memory.counter = memory.data[begin] & 0x7F
  else:
    memory.counter = 0

def find_tail():
  """Find the tail (read) index"""
  begin = 0
  end = len(memory.data)
  while begin + 1 != end:
    setmiddle = begin + (end - begin) / 2
    middle = setmiddle - 1
    last = end - 1
    firsthalf = False
    # Check if there is data in the last sector
    if (memory.data[last] & 0x80) == 0 or memory.data[last] == 0xFF:
      # No data in last sector, check if there's data in the middle sector
      if (memory.data[middle] & 0x80) == 0 or memory.data[middle] == 0xFF:
        # No data in middle sector, check the sequence
        firsthalf = memory.data[middle] == 0xFF or \
                    (memory.data[begin] & 0x7F) >= (memory.data[middle] & 0x7F)
      else:
        # Data in middle sector, check the sequence
        firsthalf = (memory.data[middle] & 0x7F) >= (memory.data[begin] & 0x7F)
    else:
      # Data in last sector, check if there's data in the middle sector
      if (memory.data[middle] & 0x80) == 0 or memory.data[middle] == 0xFF:
        # No data in middle sector, tail is in second half
        pass
      else:
        # Data in middle sector, check the sequence
        firsthalf = (memory.data[last] & 0x7F) >= (memory.data[middle] & 0x7F)
    # Now adjust the right boundary
    if firsthalf:
      end = setmiddle
    else:
      begin = setmiddle
  memory.tail = begin

def start_flash():
  """Simulate flash start-up to find head and tail"""
  find_head()
  find_tail()
  print_data()

def write_sector():
  """Simulate a new sector write"""
  if memory.data[memory.head] == 0xFF:
    newhead = memory.head
  else:
    newhead = next(memory.head)
  if (memory.data[newhead] & 0x80) and memory.data[newhead] != 0xFF:
    print "Memory is full"
    print_data()
    return
  memory.head = newhead
  memory.counter += 1
  memory.data[memory.head] = memory.counter | 0x80
  print_data()

def read_sector():
  """Simulate a sector read"""
  if memory.data[memory.tail] == 0xFF or \
      (memory.data[memory.tail] & 0x80) == 0:
    print "Memory is empty"
    print_data()
    return
  memory.data[memory.tail] &= ~0x80
  if memory.tail == memory.head:
    memory.head = next(memory.head)
    memory.counter += 1
    memory.data[memory.head] = memory.counter | 0x80
  memory.tail = next(memory.tail)
  print_data()

def erase_sector():
  """Simulate a sector erase, this allows us to test whether the head and
  tail search algorithms work if the when a sector is erased and the power
  is removed before a new sector sequence number is written."""
  erase = next(memory.head)
  if (memory.data[erase] & 0x80) and memory.data[erase] != 0xFF:
    print "Memory is full"
    print_data()
    return
  memory.data[erase] = 0xFF
  print_data()

