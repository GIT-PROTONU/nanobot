// Project-owned zenoh-pico feature config, used when -DZENOH_GENERIC is set (see
// zenoh-pico/include/zenoh-pico/config.h). This replaces the CMake-generated block
// so our settings survive a lib re-fetch (PlatformIO doesn't run zenoh-pico's CMake,
// and the shipped config.h hard-#defines Z_FEATURE_LINK_SERIAL 0).
//
// Only change vs. the stock arduino-esp32 defaults: Z_FEATURE_LINK_SERIAL = 1, so the
// `serial/<tx>.<rx>#baudrate=...` locator schema is registered (otherwise z_open
// returns -92 _Z_ERR_CONFIG_LOCATOR_SCHEMA_UNKNOWN).
#ifndef NANO_ZENOH_GENERIC_CONFIG_H
#define NANO_ZENOH_GENERIC_CONFIG_H

#define Z_FRAG_MAX_SIZE 4096
#define Z_BATCH_UNICAST_SIZE 2048
#define Z_BATCH_MULTICAST_SIZE 2048
#define Z_CONFIG_SOCKET_TIMEOUT 100
#define Z_TRANSPORT_LEASE 10000
#define Z_TRANSPORT_LEASE_EXPIRE_FACTOR 3
#define Z_RUNTIME_MAX_TASKS 64
#define Z_TRANSPORT_ACCEPT_TIMEOUT 1000
#define Z_TRANSPORT_CONNECT_TIMEOUT 10000

#define Z_FEATURE_CONNECTIVITY 0
// Multi-thread: zenoh-pico's serial read BLOCKS until a full frame arrives, so a
// dedicated read task must own RX while the lease task + our publishes do TX. TX is
// serialized by Z_FEATURE_BATCH_TX_MUTEX so concurrent writes don't corrupt frames.
// These tasks + our Core-1 control loop run in parallel across both ESP32 cores.
#define Z_FEATURE_MULTI_THREAD 1
#define Z_FEATURE_PUBLICATION 1
#define Z_FEATURE_ADVANCED_PUBLICATION 0
#define Z_FEATURE_SUBSCRIPTION 1
#define Z_FEATURE_ADVANCED_SUBSCRIPTION 0
#define Z_FEATURE_QUERY 1
#define Z_FEATURE_QUERYABLE 1
#define Z_FEATURE_LIVELINESS 1
#define Z_FEATURE_RAWETH_TRANSPORT 0
#define Z_FEATURE_INTEREST 1
#define Z_FEATURE_LINK_TCP 1
#define Z_FEATURE_LINK_BLUETOOTH 0
#define Z_FEATURE_LINK_WS 0
#define Z_FEATURE_LINK_SERIAL 1
#define Z_FEATURE_LINK_SERIAL_USB 0
#define Z_FEATURE_LINK_TLS 0
#define Z_FEATURE_SCOUTING 1
#define Z_FEATURE_LINK_UDP_MULTICAST 1
#define Z_FEATURE_LINK_UDP_UNICAST 1
#define Z_FEATURE_MULTICAST_TRANSPORT 1
#define Z_FEATURE_UNICAST_TRANSPORT 1
#define Z_FEATURE_FRAGMENTATION 1
#define Z_FEATURE_ENCODING_VALUES 1
#define Z_FEATURE_TCP_NODELAY 1
#define Z_FEATURE_LOCAL_SUBSCRIBER 0
#define Z_FEATURE_LOCAL_QUERYABLE 0
#define Z_FEATURE_SESSION_CHECK 1
#define Z_FEATURE_BATCHING 1
#define Z_FEATURE_BATCH_TX_MUTEX 1
#define Z_FEATURE_BATCH_PEER_MUTEX 0
#define Z_FEATURE_MATCHING 1
#define Z_FEATURE_RX_CACHE 0
#define Z_FEATURE_UNICAST_PEER 1
#define Z_FEATURE_AUTO_RECONNECT 1
#define Z_FEATURE_MULTICAST_DECLARATIONS 0
#define Z_FEATURE_ADMIN_SPACE 0

#endif
