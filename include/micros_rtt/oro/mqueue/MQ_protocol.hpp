#ifndef MICROSRTT_MQ_TEMPATE_PROTOCOL_BASE_HPP
#define MICROSRTT_MQ_TEMPATE_PROTOCOL_BASE_HPP

#include "MQLib.hpp"
#include "../../types/TypeMarshaller.hpp"
#include "MQChannelElement.hpp"

#include <boost/type_traits/has_virtual_destructor.hpp>
#include <boost/static_assert.hpp>

namespace micros_rtt
{ namespace mqueue
  {
      /**
       * For each transportable type T, specify the conversion functions.
       * @warning This can only be used if T is a trivial type without
       * meaningful (copy) constructor. For all other cases, or in doubt,
       * use the MQSerializationProtocol class.
       *
       */
      template<class T>
      class MQTemplateProtocolBase
          : public RTT::types::TypeMarshaller
      {
      public:
          /**
           * The given \a T parameter is the type for reading DataSources.
           */
          typedef T UserType;

          virtual base::ChannelElementBase::shared_ptr createStream(base::PortInterface* port, const ConnPolicy& policy, bool is_sender) const {
              try {
                  base::ChannelElementBase::shared_ptr mq = new MQChannelElement<T>(port, *this, policy, is_sender);
                  if ( !is_sender ) {
                      // the receiver needs a buffer to store his messages in.
                      base::ChannelElementBase::shared_ptr buf = detail::DataSourceTypeInfo<T>::getTypeInfo()->buildDataStorage(policy);
                      mq->setOutput(buf);
                  }
                  return mq;
              } catch(std::exception& e) {
                  log(Error) << "Failed to create MQueue Channel element: " << e.what() << endlog();
              }
              return base::ChannelElementBase::shared_ptr();
          }

      };
}
}

#endif

