PULSE Flash Imaging
===================

This document describes the PULSE flash imaging protocol. This protocol
was originally designed for use over PULSEv1, but also works over the
PULSEv2 Best-Effort transport.

The flash imaging protocol is used to write raw data directly to the
external flash. It is a message-oriented protocol encapsulated in PULSE
frames. The primary goals of the protocol are reliability and speed over
high-latency links.

* All client-sent commands elicit responses

* As much as possible, any command can be resent without corrupting the
  flashing process. This is to accommodate the situation where the
  command was received and acted upon but the response was lost, and the
  client retried the command.

* Any notification (server→client message which is not a response to a
  command) can be lost without holding up the flashing process. There
  must be a way to poll for the status of all operations which elicit
  notifications.

* Most of the state is tracked by the client. The server only has to
  maintain a minimal, fixed-size amount of state.

> The idempotence of writing to flash is leveraged in the design of this
> protocol to effectively implement a [Selective Repeat ARQ](https://en.wikipedia.org/wiki/Selective_Repeat_ARQ)
> with an unlimited window size without requiring the server to keep
> track of which frames are missing. Any Write Data command to the same
> location in flash can be repeated any number of times with no ill
> effects.

## Message format

All fields in a message which are more than one octet in length are
transmitted least-significant octet (LSB) first.

All messages begin with a 1-octet Opcode, followed by zero or more data
fields depending on the message. All Address fields are offsets from the
beginning of flash. Address and Length fields are specified in units of
bytes.

### Client Commands

#### 1 - Erase flash region

Address: 4 octets
Length: 4 octets

#### 2 - Write data to flash

Address: 4 octets
Data: 1+ octets

The data length is implied.

#### 3 - Checksum flash region

Address: 4 octets
Length: 4 octets

#### 4 - Query flash region geometry

Region: 1 octet

Region | Description
-------|-------------------
     1 | PRF
     2 | Firmware resources

#### 5 - Finalize flash region

Region: 1 octet

Inform the server that writing is complete and perform whatever task is
necessary to finalize the data written to the region. This may be a
no-op.

Region numbers are the same as for the "Query flash region geometry"
message.

### Server Responses

#### 128 - ACKnowledge erase command

Address: 4 octets
Length: 4 octets
Complete?: 1 octet

Complete field is zero if the erase is in progress, nonzero when the
erase is complete.

#### 129 - ACKnowledge write command

Address: 4 octets
Length: 4 octets
Complete?: 1 octet

#### 130 - Checksum result

Address: 4 octets
Length: 4 octets
Checksum: 4 octets

The legacy Pebble checksum ("STM32 CRC") of the specified memory is
returned.

#### 131 - Flash region geometry

Region: 1 octet
Address: 4 octets
Length: 4 octets

A length of zero indicates that the region does not exist.

#### 132 - ACKnowledge finalize flash region command

Region: 1 octet

#### 192 - Malformed command

Bad message: 9 octets
Error string: 0+ octets

#### 193 - Internal error

Error string: 0+ octets

Something has gone terribly wrong which prevents flashing from
proceeding.


<!-- vim: set tw=72: -->
