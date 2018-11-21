#pragma once

#include <cstdint>

namespace rp { namespace slamware { namespace message {

    typedef std::uint64_t message_timestamp_t;

    template < typename TPayload >
    struct Message
    {
        Message()
			: timestamp(0)
        {
        }

        Message(const TPayload& that)
            : timestamp(0)
			, payload(that)
        {
        }

        message_timestamp_t timestamp;
        TPayload payload;

        inline TPayload& operator*()
        {
            return payload;
        }

        inline TPayload* operator->()
        {
            return &payload;
        }

        inline const TPayload& operator*() const
        {
            return payload;
        }
        
        inline const TPayload* operator->() const
        {
            return &payload;
        }

        Message<TPayload>& operator=(const TPayload& that)
        {
            payload = that;
            return *this;
        }
    };

} } }
