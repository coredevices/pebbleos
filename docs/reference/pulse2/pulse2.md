PULSEv2 Protocol Suite
======================

Motivation
----------

The initial design of PULSE was shaped by its initial use case of flash
imaging. Flash imaging has a few properties which allowed it to be
implemented on top of a very simplistic wire protocol. Writing to flash
can be split up into any number of atomic write operations that can be
applied in arbitrary order. Flash writes are idempotent: repeatedly
writing the same data to the same flash address does not corrupt the
written data. Because of these properties, it was possible to implement
the flash imaging protocol in a stateless manner simply by ensuring that
every write was applied at least once without concern for out of order
delivery or duplicated datagrams. The PULSE link layer was designed as
simply as possible, guaranteeing only datagram integrity with only
best-effort reliability and sequencing, since it was all that the flash
imaging protocol needed.

As we try to use PULSE for more applications, it has become clear that
flash imaging is a special case. Most applications have some manner of
statefulness or non-idempotent operations, so they need guarantees about
reliable delivery and sequencing of datagrams in order to operate
correctly in the face of lost or corrupted datagrams. The lack of such
guarantees in PULSE has forced these applications to bake sequencing and
retransmissions into the application protocols in an ad-hoc manner,
poorly. This has made the design and implementation of prompt and file
transfer protocols more complex than necessary, and no attempt has yet
been made to tunnel Pebble Protocol over PULSE. It's the [waterbed
theory](https://wiki.c2.com/?WaterbedTheory) at work.

Adding support for reliable, ordered delivery of datagrams will allow
for any application to make use of reliable service simply by requesting
it. Implementation of chatty protocols will be greatly simplified.

Protocol Stack
--------------

PULSEv2 is a layered protocol stack. The link layer provides
integrity-assured delivery of packet data. On top of the link layer is a
suite of transport protocols which provide multiprotocol delivery of
application datagrams with or without guaranteed reliable in-order
delivery. Application protocols use one or more of the available
transports to exchange datagrams between the firmware running on a board
and a host workstation.

Physical Layer
--------------

PULSEv2 supports asynchronous serial byte-oriented full duplex links,
8-N-1, octets transmitted LSB first. The link must transparently pass
all octet values. The baud rate is 1,000,000 bps.

> **Why that baud rate?**
>
> 1 Mbaud is a convenient choice as it is the highest frequency which
> divides perfectly into a 16 MHz core clock at 16x oversampling, and
> works with zero error at 64, 80 and 100 MHz (with only 100 MHz
> requiring any fractional division at all). The only downside is that
> it is not a "standard" baud rate, but this is unlikely to be a problem
> as FTDI, PL2303, CP2102 (but not CP2101) and likely others will handle
> 1 Mbaud rates (at least in hardware). YMMV with Windows drivers...

Link Layer
----------

The link layer, in a nutshell, is PPP with custom framing. The entirety
of [RFC 1661](https://tools.ietf.org/html/rfc1661) is normative, except
as noted in this document.

### Encapsulation

PPP encapsulation (RFC 1661, Section 2) is used. The Padding field of
the PPP encapsulation must be empty.

A summary of the frame structure is shown below. This figure does not
include octets inserted for transparency. The fields are transmitted
from left to right.

| Flag | Protocol | Information | FCS      | Flag |
| ---- | -------- | ----------- | -------- | ---- |
| 0x55 | 2 octets | *           | 4 octets | 0x55 |

#### Flag field

Each frame begins and ends with a Flag sequence, which is the octet 0x55
hexadecimal. The flag is used for frame synchronization.

> **Why 0x55?**
>
> It is transmitted as bit pattern `(1)0101010101`, which is really easy
> to spot on an oscilloscope trace or logic analyzer capture, and it
> allows for auto baud rate detection. The STM32F7 USART supports auto
> baud rate detection with an 0x55 character in hardware.

Only one Flag sequence is required between two frames. Two consecutive
Flag sequences constitute and empty frame, which is silently discarded.

#### Protocol field

The Protocol field is used as prescribed by RFC 1661, Section 2. PPP
assigned protocol numbers and their respective assigned protocols should
be used wherever it makes sense. Custom protocols must not be assigned
protocol numbers which overlap any [existing PPP assigned protocol](https://www.iana.org/assignments/ppp-numbers/ppp-numbers.xhtml).

#### Frame Check Sequence field

The Frame Check Sequence is transmitted least significant octet first.
The check sequence is calculated using the [CRC-32](https://reveng.sourceforge.net/crc-catalogue/all.htm#crc.cat.crc-32)
checksum. The parameters of the CRC algorithm are:

    width=32 poly=0x04c11db7 init=0xffffffff refin=true refout=true
    xorout=0xffffffff check=0xcbf43926 name="CRC-32"

The FCS field is calculated over all bits of the Protocol and
Information fields, not including any start and stop bits, or octets
inserted for transparency. This also does not include the Flag sequence
nor the FCS field itself.

### Transparency

Transparency is achieved by applying [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing)
encoding to the Protocol, Information and FCS fields, then replacing any
instances of 0x55 in the COBS-encoded data with 0x00.

### Link Operation

The Link Control Protocol packet format, assigned numbers and state
machine are the same as PPP (RFC 1661), with minor exceptions.

> Do not be put off by the length of the RFC document. Only a small
> subset of the protocol needs to be implemented (especially if there
> are no negotiable options) for an implementation to be conforming.

> All multi-byte fields in LCP packets are transmitted in Network
> (big-endian) byte order. The burden of converting from big-endian to
> little-endian is very minimal, and it lets Wireshark dissectors work
> on PULSEv2 LCP packets just like any other PPP LCP packet.

By prior agreement, peers MAY transmit or receive packets of certain
protocols while the link is in any phase. This is contrary to the PPP
standard, which requires that all non-LCP packets be rejected before the
link reaches the Authentication phase.

Transport Layer
---------------

### Best-Effort Application Transport (BEAT) protocol

Best-effort delivery with very little overhead, similar to PULSEv1.

#### Packet format

| Application Protocol | Length | Information |
| -------------------- | ------ | ----------- |
| 2 octets             | 2 octets | *         |

All multibyte fields are in big-endian byte order.

The Length field encodes the number of octets in the Application
Protocol, Length and Information fields of the packet. The minimum value
of the Length field in a valid packet is 4.

BEAT application protocol 0x0001 is assigned to the PULSE Control
Message Protocol (PCMP). When a BEAT packet is received by a conforming
implementation with the Application Protocol field set to an
unrecognized value, a PCMP Unknown Protocol message MUST be sent.

#### BEAT Control Protocol (BECP)

BECP uses the same packet exchange mechanism as the Link Control
Protocol. BECP packets may not be exchanged until LCP is in the Opened
state. BECP packets received before this state is reached should be
silently discarded.

BECP is exactly the same as the Link Control Protocol with the following
exceptions:

- Exactly one BECP packet is encapsulated in the Information field of
  Link Layer frames where the Protocol field indicates type 0xBA29 hex.

- Only codes 1 through 7 (Configure-Request, Configure-Ack,
  Configure-Nak, Configure-Reject, Terminate-Request, Terminate-Ack and
  Code-Reject) are used. Other codes should be treated as unrecognized
  and should result in Code-Rejects.

- A distinct set of configure options are used. There are currently no
  options defined.

#### Sending BEAT packets

Before any BEAT protocol packets may be communicated, both LCP and BECP
must reach the Opened state. Exactly one BEAT protocol packet is
encapsulated in the Information field of Link Layer frames where the
Protocol field indicates type 0x3A29 hex.

### PUSH (Simplex) transport

Simplex best-effort delivery of datagrams. It is designed for log
messages and other status updates from the firmware to the host. There
is no NCP, no options, no negotiation.

#### Packet format

| Application Protocol | Length | Information |
| -------------------- | ------ | ----------- |
| 2 octets             | 2 octets | *         |

All multibyte fields are in big-endian byte order.

The Length field encodes the number of octets in the Application
Protocol, Length and Information fields of the packet. The minimum value
of the Length field in a valid packet is 4.

#### Sending PUSH packets

Packets can be sent at any time regardless of the state of the link,
including link closed. Exactly one PUSH packet is encapsulated in the
Information field of Link Layer frames where the Protocol field
indicates type 0x5021 hex.

### Reliable transport (TRAIN)

The Reliable transport provides reliable in-order delivery service of
multiprotocol application datagrams. The protocol is heavily based on
the [ITU-T Recommendation X.25](https://www.itu.int/rec/T-REC-X.25-199610-I/en)
LAPB data-link layer. The remainder of this section relies heavily on
the terminology used in Recommendation X.25. Readers are also assumed to
have some familiarity with section 2 of the Recommendation.

#### Packet formats

The packet format is, in a nutshell, LAPB in extended mode carrying BEAT
packets.

**Information command packets**

| Control  | Application Protocol | Length   | Information |
| -------- | -------------------- | -------- | ----------- |
| 2 octets | 2 octets             | 2 octets | *           |

**Supervisory commands and responses**

| Control  |
| -------- |
| 2 octets |

##### Control field

The control field is basically the same as LAPB in extended mode. Only
Information transfer and Supervisory formats are supported. The
Unnumbered format is not used as such signalling is performed
out-of-band using the TRCP control protocol. The Information command and
the Receive Ready, Receive Not Ready, and Reject commands and responses
are permitted in the control field.

The format and meaning of the subfields in the Control field are
described in ITU-T Recommendation X.25.

##### Application Protocol field

The protocol number for the message contained in the Information field.
This field is only present in Information packets. The Application
Protocol field is transmitted most-significant octet first.

##### Length field

The Length field specifies the number of octets covering the Control,
Application Protocol, Length and Information fields. The Length field is
only present in Information packets. The content of a valid Information
packet must be no less than six. The Length field is transmitted
most-significant octet first.

##### Information field

The application datagram itself. This field is only present in
Information packets.

#### TRAIN Control Protocol

The TRAIN Control Protocol, or TRCP for short, is used to set up and
tear down the communications channel between the two peers. TRCP uses
the same packet exchange mechanism as the Link Control Protocol. TRCP
packets may not be exchanged until LCP is in the Opened state. TRCP
packets received before this state is reached should be silently
discarded.

TRCP is exactly the same as the Link Control Protocol with the following
exceptions:

- Exactly one TRCP packet is encapsulated in the Information field of
  Link Layer frames where the Protocol field indicates type 0xBA33 hex.

- Only codes 1 through 7 (Configure-Request, Configure-Ack,
  Configure-Nak, Configure-Reject, Terminate-Request, Terminate-Ack and
  Code-Reject) are used. Other codes should be treated as unrecognized
  and should result in Code-Rejects.

- A distinct set of configure options are used. There are currently no
  options defined.

The `V(S)` and `V(R)` state variables shall be reset to zero when the
TRCP automaton signals the This-Layer-Up event. All packets in the TRAIN
send queue are discarded when the TRCP automaton signals the
This-Layer-Finished event.

#### LAPB system parameters

The LAPB system parameters used in information transfer have the default
values described below. Some parameter values may be altered through the
TRCP option negotiation mechanism. (NB: there are currently no options
defined, so there is currently no way to alter the default values during
the protocol negotiation phase)

**Maximum number of bits in an I packet _N1_** is equal to eight times
the MRU of the link, minus the overhead imposed by the Link Layer
framing and the TRAIN header. This parameter is not negotiable.

**Maximum number of outstanding I packets _k_** defaults to 1 for both
peers. This parameter is (to be) negotiable. If left at the default, the
protocol will operate with a Stop-and-Wait ARQ.

#### Transfer of application datagrams

Exactly one TRAIN packet is encapsulated in the Information field of
Link Layer frames. A command packet is encapsulated in a Link Layer
frame where the Protocol field indicates 0x3A33 hex, and a response
packet is encapsulated in a Link Layer frame where the Protocol field
indicates 0x3A35 hex. Transfer of datagrams shall follow the procedures
described in Recommendation X.25 §2.4.5 _LAPB procedures for information
transfer_. A cut-down set of procedures for a compliant implementation
which only supports _k=1_ operation can be found in
[reliable-transport.md](reliable-transport.md).

In the event of a frame rejection condition (as defined in
Recommendation X.25), the TRCP automaton must be issued a Down event
followed by an Up event to cause an orderly renegotiation of the
transport protocol and reset the state variables. This is the same as
the Restart option described in RFC 1661. A FRMR response MUST NOT be
sent.

TRAIN application protocol 0x0001 is assigned to the PULSE Control
Message Protocol (PCMP). When a TRAIN packet is received by a conforming
implementation with the Application Protocol field set to an
unrecognized value, a PCMP Unknown Protocol message MUST be sent.

### PULSE Control Message Protocol

The PULSE Control Message Protocol (PCMP) is used for signalling of
control messages by the transport protocols. PCMP messages must be
encapsulated in a transport protocol, and are interpreted within the
context of the encapsulated transport protocol.

> **Why a separate protocol?**
>
> Many of the transports need to communicate the same types of control
> messages. Rather than defining a different way of communicating these
> messages for each protocol, they can use PCMP and share a single
> definition (and implementation!) of these messages.

#### Packet format

| Code    | Information |
| ------- | ----------- |
| 1 octet | *           |

#### Defined codes

##### 1 - Echo Request

When the transport is in the Opened state, the recipient MUST respond
with an Echo-Reply packet. When the transport is not Opened, any
received Echo-Request packets MUST be silently discarded.

##### 2 - Echo Reply

A reply to an Echo-Request packet. The Information field MUST be copied
from the received Echo-Request.

##### 3 - Discard Request

The receiver MUST silently discard any Discard-Request packet that it
receives.

##### 129 - Port Closed

A packet has been received with a port number unrecognized by the
recipient. The Information field must be filled with the port number
copied from the received packet (without endianness conversion).

##### 130 - Unknown PCMP Code

A PCMP packet has been received with a Code field which is unknown to
the recipient. The Information field must be filled with the Code field
copied from the received packet.

---

Useful Links
------------

- [The design document for PULSEv2](https://docs.google.com/a/pulse-dev.net/document/d/1ZlSRz5-BSQDsmutLhUjiIiDfVXTcI53QmrqENJXuCu4/edit?usp=sharing),
  which includes a draft of this documentation along with a lot of
  notes about the design decisions.
- [Python implementation of PULSEv2](https://github.com/coredevices/PebbleOS/tree/main/tools/libs/pulse2)
- [RFC 1661 - The Point to Point Protocol (PPP)](https://tools.ietf.org/html/rfc1661)
- [RFC 1662 - PPP in HDLC-like Framing](https://tools.ietf.org/html/rfc1662)
- [RFC 1663 - PPP Reliable Transmission](https://tools.ietf.org/html/rfc1663)
- [RFC 1570 - PPP LCP Extensions](https://tools.ietf.org/html/rfc1570)
- [RFC 2153 - PPP Vendor Extensions](https://tools.ietf.org/html/rfc2153)
- [RFC 3772 - Point-to-Point Protocol (PPP) Vendor Protocol](https://tools.ietf.org/html/rfc3772)
- [PPP Consistent Overhead Byte Stuffing (COBS)](https://tools.ietf.org/html/draft-ietf-pppext-cobs)
- [ITU-T Recommendation X.25](https://www.itu.int/rec/T-REC-X.25-199610-I/en)
- [Digital Data Communications Message Protocol](https://www.ibiblio.org/pub/historic-linux/early-ports/Mips/doc/DEC/ddcmp-4.1.txt)

<!-- vim: set tw=72: -->
