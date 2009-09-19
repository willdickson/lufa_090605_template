"""
-----------------------------------------------------------------------
simple_step
Copyright (C) William Dickson, 2008.
  
wbd@caltech.edu
www.willdickson.com

Released under the LGPL Licence, Version 3

This file is part of simple_step.

simple_step is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
    
simple_step is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with simple_step.  If not, see
<http://www.gnu.org/licenses/>.

------------------------------------------------------------------------

Purpose: Provides and API for the at90usb based stepper motor
controller. 

Author: William Dickson 

------------------------------------------------------------------------
"""
import pylibusb as usb
import ctypes
import sys
import time
import math
import struct

DEBUG = False 
INT32_MAX = (2**32-1)/2

# USB parameters
USB_VENDOR_ID = 0x0004 
USB_PRODUCT_ID = 0x0001
USB_BULKOUT_EP_ADDRESS = 0x01
USB_BULKIN_EP_ADDRESS = 0x82
USB_BUFFER_OUT_SIZE = 64 
USB_BUFFER_IN_SIZE = 64
USB_DATA_PACKET_SIZE = 30

# USB Command IDs
USB_CMD_TEST8 = 0
USB_CMD_TEST16 = 1
USB_CMD_TEST32 = 2
USB_CMD_TEST_SET = 3
USB_CMD_TEST_GET = 4
USB_CMD_AVR_RESET = 200
USB_CMD_AVR_DFU_MODE = 201

# Maximum data array lenghts for given types
DATAARRAY_MAX_SIZE = 60

# C data types
c_int8 = ctypes.c_int8
c_int16 = ctypes.c_int16
c_int32 = ctypes.c_int32
c_uint8 = ctypes.c_uint8
c_uint16 = ctypes.c_uint16
c_uint32 = ctypes.c_uint32
c_float = ctypes.c_float
c_double = ctypes.c_double
c_char = ctypes.c_char

# Useful ctypes functions
sizeof = ctypes.sizeof
cast = ctypes.cast
POINTER = ctypes.POINTER

BYTES_PER_TYPE= {
        'int8_t'   : 1,
        'uint8_t'  : 1,
        'int16_t'  : 2,
        'uint16_t' : 2,
        'int32_t'  : 4,
        'uint32_t' : 4,
        }

def debug(val):
    if DEBUG==True:
        print >> sys.stderr, val

def debug_print(msg, comma=False):
    if DEBUG==True:
        if comma==True:
            print msg, 
        else:
            print msg
        sys.stdout.flush()

class USB_Device:

    """
    Example USB interface to the at90usb based stepper motor controller board.
    """

    def __init__(self,serial_number=None):
        """
        Open and initialize usb device.
        
        Arguments: None
        
        Return: None.
        """
        usb.init()

        #usb.set_debug(3)
        
        # Get usb busses
        if not usb.get_busses():
            usb.find_busses()            
            usb.find_devices()
        busses = usb.get_busses()

        # Find device by IDs
        found = False
        dev_list = []
        for bus in busses:
            for dev in bus.devices:
                if (dev.descriptor.idVendor == USB_VENDOR_ID and
                    dev.descriptor.idProduct == USB_PRODUCT_ID):
                    dev_list.append(dev)
                    found = True
                    #break
            #if found:
            #    break
        if not found:
            raise RuntimeError("Cannot find device.")

        if serial_number == None:
            # No serial number specified - take first device
            dev = dev_list[0]
            self.libusb_handle = usb.open(dev)
            self.dev = dev
        else:
            # Try and find device with specified serial number
            found = False
            for dev in dev_list:
                self.dev = dev
                self.libusb_handle = usb.open(dev)
                sn = self.get_serial_number()
                if sn == serial_number:
                    found = True
                    break
                else:
                    ret = usb.close(self.libusb_handle)
            if not found:
                raise RuntimeError("Cannot find device w/ serial number %s."%(serial_number,))

        self.interface_nr = 0
        if hasattr(usb,'get_driver_np'):
            # non-portable libusb function available
            name = usb.get_driver_np(self.libusb_handle,self.interface_nr)
            if name != '':
                debug("attached to kernel driver '%s', detaching."%name )
                usb.detach_kernel_driver_np(self.libusb_handle,self.interface_nr)

        if dev.descriptor.bNumConfigurations > 1:
            debug("WARNING: more than one configuration, choosing first")

        usb.set_configuration(self.libusb_handle, self.dev.config[0].bConfigurationValue)
        usb.claim_interface(self.libusb_handle, self.interface_nr)

        self.output_buffer = ctypes.create_string_buffer(USB_BUFFER_OUT_SIZE)
        self.input_buffer = ctypes.create_string_buffer(USB_BUFFER_IN_SIZE)
        for i in range(USB_BUFFER_IN_SIZE):
            self.input_buffer[i] = chr(0x00)
        for i in range(USB_BUFFER_OUT_SIZE):
            self.output_buffer[i] = chr(0x00)

        # Clear any possible halt on the endpoints
        ret = usb.clear_halt(self.libusb_handle,USB_BULKOUT_EP_ADDRESS)
        ret = usb.clear_halt(self.libusb_handle,USB_BULKIN_EP_ADDRESS)


    def close(self):
        """
        Close usb device.
        
        Arguments: None
        
        Return: None
        """
        ret = usb.release_interface(self.libusb_handle,self.interface_nr)
        ret = usb.close(self.libusb_handle)
        return

    # -------------------------------------------------------------------------
    # Methods for low level USB communication 
        
    def __send_and_receive(self,in_timeout=200,out_timeout=9999):
        """
        Send bulkout and and receive bulkin as a response.
        
        Arguments: None
        
        Keywords: 
          in_timeout  = bulkin timeout in ms
          out_timeout = bilkin timeout in ms
          
        Return: the data returned by the usb device.
        """
        done = False
        while not done:
            val = self.__send_output(timeout=out_timeout)
            if val < 0 :
                raise IOError, "error sending usb output"

            data = self.__read_input(timeout=in_timeout)
            if data == None:
                debug_print('usb SR: fail', comma=False) 
                sys.stdout.flush()
                continue
            else:
                done = True
                debug_print('usb SR cmd_id: %d'%(ord(data[0]),), comma=False) 

        return data

    def __send_output(self,timeout=9999):
        """
        Send output data to the usb device.
        
        Arguments: None
        
        Keywords:
          timeout = the timeout in ms
          
        Return: number of bytes written on success or < 0 on error.
        """
        buf = self.output_buffer # shorthand
        val = usb.bulk_write(self.libusb_handle, USB_BULKOUT_EP_ADDRESS, buf, timeout)
        return val

    def __read_input(self, timeout=1000):
        """
        Read input data from the usb device.
        
        Arguments: None
        
        Keywords:
          timeout = the timeout in ms
          
        Return: the raw data read from the usb device.
        """
        buf = self.input_buffer
        try:
            val = usb.bulk_read(self.libusb_handle, USB_BULKIN_EP_ADDRESS, self.input_buffer, timeout)
            data = [x for x in self.input_buffer]
        except usb.USBNoDataAvailableError:
            data = None
        return data
        
    def __int_to_bytes(self,val,int_type):
        """
        Convert integer value to bytes based on type specifier.

        Arguments:
          val      = the integer value to convert
          int_type = the integer type specifier
                     'uint8_t'  = unsigned 8 bit integer
                     'uint16_t' = unsigned 16 bit integer
                     'int32_t'  = signed 32 bit integer
                     
        Return: the integer converted to bytes.
        """
        int_type = int_type.lower()
        if int_type in ('int8_t', 'uint8_t'):
            bytes = [chr(val&0xFF)]
        elif int_type in ('int16_t', 'uint16_t'):
            bytes = [chr(val&0xFF), chr((val&0xFF00)>>8)]
        elif int_type in ('int32_t', 'uint32_t'):
            bytes = [chr((val&0xFF)),
                     chr((val&0xFF00) >> 8),
                     chr((val&0xFF0000) >> 16),
                     chr((val&0xFF000000) >> 24),]
        else:
            raise ValueError, "unknown int_type %s"%(int_type,)
        return bytes
        
    def __bytes_to_int(self,bytes,int_type):
        """
        Convert sequence of  bytes to intN or uintN based on the type
        specifier.

        Arguments:
          bytes    = the bytes to convert
          int_type = the integer type specifier
                     'uint8_t'  = unsigned 8 bit integer
                     'uint16_t' = unsigned 16 bit integer
                     'int32_t'  = signed 32 bit integer
        
        Return: the integer value
        """
        int_type = int_type.lower()
        if int_type == 'uint8_t':
            # This is unsigned 8 bit integer
            val = ord(bytes[0])
            #y = cast(self.input_buffer[2],POINTER(c_uint8)).contents.value
            #print y
        elif int_type == 'uint16_t':
            # This is unsigned 16 bit integer
            val = ord(bytes[0]) 
            val += ord(bytes[1]) << 8
            #y = cast(self.input_buffer[2:4],POINTER(c_uint16)).contents.value
            #print y
        elif int_type == 'uint32_t':
            # This is unsigned 32 bit integer
            val = ord(bytes[0]) 
            val += ord(bytes[1]) << 8
            val += ord(bytes[2]) << 16
            val += ord(bytes[3]) << 24
            #y = cast(self.input_buffer[2:6],POINTER(c_uint32)).contents.value
            #print y
        elif int_type == 'int32_t':
            # This is signed 32 bit integer
            val = ord(bytes[0]) 
            val += ord(bytes[1]) << 8
            val += ord(bytes[2]) << 16
            val += ord(bytes[3]) << 24
            if val > INT32_MAX:
                # Reverse twos complement for negatives
                val = val - 2**32
        else:
            raise ValueError, "unknown int_type %s"%(int_type,)
        return val

    def __write_to_buffer(self, cmd_id, outdata):
        """
        Write data list/array to output buffer.
        """

        # Check size of data array
        N = 0 
        for d, dtype in outdata:
            N += BYTES_PER_TYPE[dtype]

        if N > DATAARRAY_MAX_SIZE:
            raise ValueError, 'data array larger than max length'

        # Empty output buffer
        for i in range(USB_BUFFER_OUT_SIZE):
            self.output_buffer[i] = chr(0x00)

        # Add command id and data array lenght to output buffer
        self.output_buffer[0] = chr(cmd_id%0x100)
        self.output_buffer[1] = self.__int_to_bytes(N,'uint8_t')[0]

        # Add data array to output buffer
        p = 2
        for d, dtype in outdata:
            bytes = self.__int_to_bytes(d,dtype)
            for b in bytes:
                self.output_buffer[p] = b
                p += 1

    def __extract_input_data(self, data, intypes):
        """
        Read data from input buffer.
        """
        cmd_id = self.__bytes_to_int(data[0],'uint8_t')
        length = self.__bytes_to_int(data[1],'uint8_t')

        N = 0 
        for dtype in intypes:
            N += BYTES_PER_TYPE[dtype]

        if length != N:
            raise IOError, 'returned bytes not equal to bytes specified by intypes'

        p = 2 
        val_list = []
        for dtype in intypes:
            if dtype in ('int8_t', 'uint8_t'):
                val = self.__bytes_to_int(data[p:p+1],dtype)
                p += 1
            elif dtype in ('int16_t', 'uint16_t'):
                val = self.__bytes_to_int(data[p:p+2],dtype)
                p += 2
            elif dtype in ('int32_t', 'uint32_t'):
                val = self.__bytes_to_int(data[p:p+4],dtype)
                p += 4
            else:
                raise ValueError, 'unknown dtype %s'%(dtype,)
            val_list.append(val)
        return val_list

    def __read_from_buffer(self,input_ctypes):
        val_list = []
        pos = 0
        for ctype in input_ctypes:
            bytes = sizeof(ctype)
            val = cast(self.input_buffer[pos:pos+bytes],POINTER(ctype)).contents.value
            pos += bytes
            val_list.append(val)
        return val_list

    #def __write_to_buffer(self,output_data):
    #    for val, ctype in output_data:
    #        pass


    def usb_cmd(self,cmd_id,outdata,intypes):
        """
        Generic usb command.
        """
        self.__write_to_buffer(cmd_id,outdata)
        data = self.__send_and_receive()
        val_list = self.__extract_input_data(data,intypes)
        return val_list        


    def get_serial_number(self):
        """
        Get serial number of device.
        
        Arguments: None

        Return: serial number of device - a string
        """
        return  usb.get_string_simple(self.libusb_handle, self.dev.descriptor.iSerialNumber)

    def get_manufacturer(self):
        """
        Get manufacturer of device

        Arguments: None

        Return: manufacturer string
        """
        return usb.get_string_simple(self.libusb_handle, self.dev.descriptor.iManufacturer)

    def get_product(self):
        """
        Get decive product string

        Arguments: None

        Return: product string
        """
        return usb.get_string_simple(self.libusb_handle, self.dev.descriptor.iProduct)

    def get_vendor_id(self):
        """
        Get device vendor ID.
        """
        return self.dev.descriptor.idVendor

    def get_product_id(self):
        """
        Get device product ID.
        """
        return self.dev.descriptor.idProduct

    # -----------------------------------------------------------------
    # Methods for USB Commands specified  by command IDs

    def enter_dfu_mode(self):
        """
        Places the at90usb device in programming mode for upgrading the 
        firmware. Note, after entering dfu mode no further communications
        with the device will be possible.

        Arguments: None
        
        Return: None
        """
        self.output_buffer[0] = chr(USB_CMD_AVR_DFU_MODE%0x100)
        val = self.__send_output()
        return

    def reset_device(self):
        """
        Resets the at90usb device. Note, currently this function has
        some problems. First, after resetting the device no further
        usb communications with the device are possible. Second, after
        reset the device occasionally fails to enumerate correctly and
        the only way I have found which fixes this is to reset the linux
        usb system. It is probably best not to use this function. 

        Arguments: None
        
        Return: None
        """
        ###############################
        # DEBUG - has issues, see above
        ###############################

        self.output_buffer[0] = chr(USB_CMD_AVR_RESET%0x100)
        val = self.__send_output()        
        self.close()
        return

            
    def print_values(self):
        """
        Prints the current device values.
        
        Arguments: None
        
        Return None.
        """
        print 
        print 'device information'
        print ' '+ '-'*35
        print '   manufacturer:', self.get_manufacturer()
        print '   product:', self.get_product()
        print '   vendor ID:', hex(self.get_vendor_id())
        print '   product ID:', hex(self.get_product_id())
        print '   serial number:',self.get_serial_number()
        
        
def check_cmd_id(expected_id,received_id):
    """
    Compares expected and received command ids.
    
    Arguments:
      expected_id = expected command id
      received_is = received command id
      
    Return: None
    """
    if not expected_id == received_id:
        msg = "received incorrect command ID %d expected %d"%(received_id,expected_id)
        raise IOError, msg
    return




#-------------------------------------------------------------------------------------
if __name__ == '__main__':

    dev = USB_Device()
    dev.print_values()
    print 

    if 0:
        N = 500
        t0 = time.time()
        for i in range(0,N):
            val = dev.usb_cmd(USB_CMD_TEST8,[],['uint8_t']*60)
        t1 = time.time()
        dt_total = t1 - t0
        dt = dt_total/float(N)
        rate = float(N)/dt_total
        print 'N:', N
        print 'dt:', dt
        print 'rate:', rate
        print

    if 0:
        for i in range(0,10):
            val = dev.usb_cmd(USB_CMD_TEST8,[],['uint8_t']*60)
            print val

    if 1:
        val_list = dev.usb_cmd(USB_CMD_TEST8,[(1,'uint8_t')],['uint8_t']*60)
        print val_list
        print
        val_list = dev.usb_cmd(USB_CMD_TEST16,[],['uint16_t']*30)
        print val_list
        print
        val_list = dev.usb_cmd(USB_CMD_TEST32,[],['uint32_t']*15)
        print val_list
        print


    if 0:
        for i in range(0,500):
            type_list = ['uint8_t', 'uint16_t','uint32_t']
            rval_list = dev.usb_cmd(USB_CMD_TEST_GET,[],type_list)
            print i
            print rval_list
            send_list = [(x+100,y) for x,y in zip(rval_list,type_list)]
            rval_list = dev.usb_cmd(USB_CMD_TEST_SET,send_list,[])
            rval_list = dev.usb_cmd(USB_CMD_TEST_GET,[],type_list)
            print rval_list
            print


    dev.close()


    
    
