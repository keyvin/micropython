

class term:
    def __init__(self, rows, columns):
        self.rows = rows
        self.columns = columns
        self.size = rows*columns
        self.data = bytearray(rows*columns)
        for i in range(len(self.data)):
            self.data[i] = 0x20
        self.x = 0
        self.y = 0

    def scroll(self):
        #copies all text up a line
        position = self.columns
        while position < self.rows*self.columns:
            self.data[position-self.columns] = self.data[position]
            position=position + 1
        for i in range(self.columns):
            self.data[self.size-1-i] = 0x20
        self.y = self.rows -1
        self.x = 0
        
    def write(self, text):
        try:
            text = bytes(text,'utf-8')
        except:
            print("Error decoding/encoding")
        for char in text:
            if char == 0x0A:
                self.y = self.y + 1
                self.x = 0
                if self.y == self.rows:
                    self.scroll()
            elif char == 0x0D:
                self.x = 0
            elif char == 0x08:
                self.data[self.x + (self.y*self.columns)] = ' '
                self.x = self.x -1
                if self.x < 0:
                    self.y = self.y - 1
                    self.x = self.columns-1
                    if self.y < 0:
                        self.y =0
                        self.x = 0
            else:
                self.data[self.x + (self.y*self.columns)] = char
                self.x = self.x+1
                if self.x == self.columns:
                    self.x = 0
                    self.y = self.y+1
                    if self.y == self.rows:
                        self.scroll()
        
            
                
    
