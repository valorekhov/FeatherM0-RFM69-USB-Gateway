package main

import (
	"bytes"
	"encoding/hex"
	"flag"
	"fmt"
	"io"
	"log"
	"os"

	"github.com/jacobsa/go-serial/serial"
)

func usage() {
	fmt.Println("rfm69-usb usage:")
	flag.PrintDefaults()
	os.Exit(-1)
}

func main() {
	fmt.Println("Staring RFM69-USB daemon")
	portName := flag.String("port", "", "serial port to test (/dev/ttyUSB0, etc)")
	flag.Parse()

	if *portName == "" {
		fmt.Println("Must specify port")
		usage()
	}

	// Set up options.
	options := serial.OpenOptions{
		PortName:        *portName, //"/dev/ttyACM0",
		BaudRate:        115200,
		DataBits:        8,
		StopBits:        1,
		MinimumReadSize: 1,
	}

	// Open the port.
	port, err := serial.Open(options)
	if err != nil {
		log.Fatalf("serial.Open: %v", err)
	} else {
		// Make sure to close it later.
		defer port.Close()
	}

	// Write 2 bytes to the port.
	cmd := []byte{0xFC, 0x00}
	n2, err2 := port.Write(cmd)
	if err2 != nil {
		log.Fatalf("port.Write: %v %d", err, n2)
	}

	command := make([]byte, 0, 4096)
	for {
		buf := make([]byte, 512)
		n, err := port.Read(buf)
		if err != nil {
			if err != io.EOF {
				fmt.Println("Error reading from serial port: ", err)
			}
		} else {
			buf = buf[:n]
			fmt.Println("Rx: ", hex.EncodeToString(buf))
			idx := bytes.IndexByte(buf, 0x0A)
			if idx >= 0 {
				command = append(command, buf[:idx+1]...)
				fmt.Println("Command:", hex.EncodeToString(command))
				command = command[:0]
			} else {
				command = append(command, buf...)
			}
		}
	}
}
