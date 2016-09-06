var ros_webrtc = ros_webrtc || {};

ros_webrtc.Signaling = Backbone.Model.extend({

  defaults: {
    server: 'ws://127.0.0.1:9000',
    id: 'me',
  },

  initialize: function () {
    this.ws = new WebSocket(this.url());
    this.ws.onopen = _.bind(this._onOpen, this);
    this.ws.onmessage = _.bind(this._onMessage, this);
    this.ws.onerror = _.bind(this._onError, this);
    this.ws.onclose = _.bind(this._onClose, this);
  },

  close: function () {
    this.stopListening();
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  },

  url: function () {
    return this.get('server') + '/' + this.get('id');
  },

  // outgoing signals

  call: function (session_id, peer_id) {
    var msg = {
          session: session_id,
          channel: peer_id,
          type: 'call'
        },
        data = JSON.stringify(msg);
    this._send(data);
  },

  pickup: function (session_id, peer_id) {
    var msg = {
          session: session_id,
          channel: peer_id,
          type: 'pickup'
        },
        data = JSON.stringify(msg);
    this._send(data);
  },

  addIceCandidate: function (session_id, peer_id, ice_candidate) {
    var msg = {
          session: session_id,
          channel: peer_id,
          type: 'add_ice_candidate',
          payload: ice_candidate
        },
        data = JSON.stringify(msg);
    this._send(data);
  },

  setSessionDescription: function (session_id, peer_id, offer_or_answer) {
    var msg = {
          session: session_id,
          channel: peer_id,
          type: 'set_session_description',
          payload: {
            type: offer_or_answer.type,
            sdp: offer_or_answer.sdp
          }
        },
        data = JSON.stringify(msg);
    this._send(data);
  },

  hangup: function (session_id, peer_id) {
    var msg = {
          session: session_id,
          channel: peer_id,
          type: 'hangup'
        },
        data = JSON.stringify(msg);
    this._send(data);
  },

  // incoming signals

  _onCall: function (msg) {
    this.trigger('call', this, msg.session, msg.channel);
  },

  _onPickup: function (msg) {
    this.trigger('pickup', this, msg.session);
  },

  _onAddIceCandidate: function (msg) {
    var ice_candidate = new RTCIceCandidate(msg.payload);
    this.trigger('add_ice_candidate', this, msg.session, ice_candidate);
  },

  _onSetSessionDescription: function (msg) {
    var sd = new RTCSessionDescription(msg.payload);
    this.trigger('set_session_description', this, msg.session, sd);
  },

  _onHangup: function (msg) {
    this.trigger('hangup', msg.session, this);
  },

  // websocket

  _send: function (data) {
    if (this.ws.readyState == WebSocket.CONNECTING) {
      this.once('connected', function () { this.ws.send(data); }, this);
    } else {
      this.ws.send(data);
    }
  },

  _onMessage: function (event) {
    var msg = JSON.parse(event.data);
    console.info('signaling recv', msg);

    var handler = {
          call: this._onCall,
          pickup: this._onPickup,
          add_ice_candidate: this._onAddIceCandidate,
          set_session_description: this._onSetSessionDescription,
          hangup: this._onHangup,
          connect: function(){},
        }[msg.type];
    handler.call(this, msg);
  },

  _onOpen: function (event) {
    console.debug('signaling connected');
    this.trigger('connected', this);
  },

  _onClose: function (event) {
    console.debug('signaling disconnected');
    this.trigger('disconnected', this);
  },

  _onError: function (event) {
    console.debug('signaling error');
    this.trigger('error', this);
  }

});

ros_webrtc.Call = Backbone.Model.extend({

  defaults: {
      connection_options: {
        optional: []
      },
      ice_servers: [{
          url: (
              navigator.mozGetUserMedia ? 'stun:stun.services.mozilla.com' :
              navigator.webkitGetUserMedia ? 'stun:stun.l.google.com:19302' :
             'stun:23.21.150.121'
          )
      }],
      media_constraints: {
          mandatory: {
              OfferToReceiveAudio: true,
              OfferToReceiveVideo: true
          }
      },
      stream: null
  },

  initialize: function () {
    // peer connection
    this.pc = new RTCPeerConnection(
        {iceServers: this.get('ice_servers')},
        this.get('connection_options')
    );
    this.pc.onaddstream = _.bind(this._pcAddStream, this);
    this.pc.onicecandidate = _.bind(this._pcIceCandidate, this);
    this.pc.ondatachannel = _.bind(this._pcDataChannel, this);
    this.queue_remote_ice_candidates = true;
    this.remote_ice_candidates = [];

    // signals from peer
    this.listenTo(this.get('signaling'), 'connected', this._signalingConnected);
    this.listenTo(this.get('signaling'), 'reconnected', this._signalingReconnected);
    this.listenTo(this.get('signaling'), 'disconnected', this._signalingDisconnected);
    this.listenTo(this.get('signaling'), 'call', this._call);
    this.listenTo(this.get('signaling'), 'pickup', this._pickup);
    this.listenTo(this.get('signaling'), 'hangup', this._hangup);
    this.listenTo(this.get('signaling'), 'add_ice_candidate', this._addIceCandidate);
    this.listenTo(this.get('signaling'), 'set_session_description', this._setSessionDescription);
  },

  dial: function () {
    this.get('signaling').call(this.get('id'), this.get('peer_id'));
  },

  pickup: function () {
    this.get('signaling').pickup(this.get('id'), this.get('peer_id'));
  },

  hangup: function () {
    this.get('signaling').hangup(this.get('id'), this.get('peer_id'));
    this.stopListening();
    this.pc.close();
  },

  adaptROSClient: function (client) {
    ros_webrtc.chunkDataChannel(client.socket);
  },

  _drainRemoteIceCandidates: function () {
    var that = this;

    function onsuccess(ice_candidate) {
      console.info( 'pc added remote ice candidate', ice_candidate);
    }

    function onerror(icecandidate, error) {
      that.trigger('error', that, error);
    }

    this.queue_remote_ice_candidates = false;
    while (this.remote_ice_candidates.length !== 0) {
      var ice_candidate = this.remote_ice_candidates.shift();
      this.pc.addIceCandidate(
        ice_candidate,
        _.partial(onsuccess, ice_candidate),
        _.partial(onerror, ice_candidate)
      );
    }
  },

  // signaling state

  _signalingConnected: function () {
    console.debug('signaling connected');
  },

  _signalingReconnected: function () {
    console.debug('signaling re-connected');
  },

  _signalingDisconnected: function () {
    console.debug('signaling disconnected');
  },

  // signals

  _call: function(signaling, session_id, peer_id) {
    console.debug('signal "call"');
    this.set('id', session_id);
    this.set('peer_id', peer_id);
  },

  _pickup: function(signaling, session_id) {
    if (this.get('id') !== session_id) {
      return;
    }
    console.debug('signal "pickup"');
    var that = this;
    this.pc.createOffer(
        function (offer) {
          offer = new RTCSessionDescription({
            type: offer.type,
            sdp: that._transformOfferSDP(offer.sdp)
          });
          that.get('signaling').setSessionDescription(
              that.get('id'), that.get('peer_id'), offer
          );
          that.pc.setLocalDescription(
              offer,
              function () {
                console.info('pc set local description', offer.sdp);
              },
              function (reason) { that.trigger('error', that, reason); }
          );
        },
        function (error) { that.trigger('error', that, error); },
        this.get('media_constraints')
    );
  },

  _hangup: function(signaling, session_id) {
    if (this.get('id') !== session_id) {
      return;
    }
    console.info('signal "hangup"');
  },

  _addIceCandidate: function (signaling, session_id, ice_candidate) {
    if (this.get('id') !== session_id) {
      return;
    }
    console.info('signal "add_ice_candidate"');
    if (this.queue_remote_ice_candidates) {
      this.remote_ice_candidates.push(ice_candidate);
      return;
    }
    var that = this;
    this.pc.addIceCandidate(
      ice_candidate,
      function () {
        console.info( 'pc added remote ice candidate', ice_candidate);
      },
      function (error) { that.trigger('error', that, error); }
    );
  },

  _setSessionDescription: function (signaling, session_id, offer_or_answer) {
    if (this.get('id') !== session_id) {
      return;
    }
    console.info('signal "set_session_description"', offer_or_answer);
    var that = this;
    this.pc.setRemoteDescription(
      offer_or_answer,
      function () {
        console.info('pc set remote description', offer_or_answer.sdp);
        if (offer_or_answer.type === 'answer') {
          that._drainRemoteIceCandidates();
          return;
        }
        that.pc.createAnswer(
          function (answer) {
            that.pc.setLocalDescription(
                answer,
                function () {
                  console.info('pc set local description', answer.sdp);
                  that._drainRemoteIceCandidates();
                },
                function (error) {
                  that.trigger('error', that, error);
                }
            );
            that.get('signaling').setSessionDescription(
              that.get('id'), that.get('peer_id'), answer
            );
          },
          function (error) { that.trigger('error', that, error); }
        );
      },
      function (error) { that.trigger('error', that, error); }
    );
  },

  // peer connection events

  _transformOfferSDP: function(sdp) {
    return sdp;
  },

  _pcAddStream: function (event) {
    console.info('pc onaddstream', event);
    this.set('stream', event.stream);
  },

  _pcIceCandidate: function (event) {
    if (!event.candidate) {
        console.info('pc onicecandidate finished', event);
        return;
    }
    console.info('pc onicecandidate', event);
    this.get('signaling').addIceCandidate(
      this.get('id'), this.get('peer_id'), event.candidate
    );
  },

  _pcDataChannel: function (event) {
    console.info('pc ondatachannel', event);
    this.trigger('datachannel', event.channel);
  },

});

ros_webrtc.ChunkedMessage = Backbone.Model.extend({

  initialize: function () {
    if (!this.has('chunks')) {
      this.set('chunks', []);
    }
  },

  addChunk: function(index, data) {
    var chunks = this.get('chunks');
      for (var i = 0; i < chunks.length; i++) {
          if (chunks.index == index) {
              chunks[i].data = data;
              return;
          }
      }
      chunks.push({index: index, data: data});
  },

  isComplete: function() {
      return this.get('chunks').length == this.get('total');
  },

  merge: function() {
    var chunks = this.get('chunks'),
        merged = "";
    chunks.sort(function (a, b) { return a.index - b.index; });
    for (var i = 0; i < chunks.length; i++) {
        merged += chunks[i].data;
    }
    return merged;
  }

});

ros_webrtc.chunkDataSize = function (dc) {
  if (!dc.protocol) {
    return 0;
  }
  var media_type = new meaty.parse(dc.protocol);
  if (!('chunksize' in media_type.parameters)) {
    return 0;
  }
  return parseInt(media_type.parameters.chunksize);
};

ros_webrtc.chunkDataChannel = function (dc) {
  var chunk_size = ros_webrtc.chunkDataSize(dc);
  if (chunk_size === 0) {
    return;
  }

  var send = dc.send,
      onmessage = dc.onmessage;

  function chunkedSend(data) {
    // http://stackoverflow.com/a/14349616
    console.info('dc chunked "' + dc.label + '" send ' + data.length + '/' + chunk_size);
    var total = Math.ceil(data.length / chunk_size);
    var offset = 0;
    var chunk = {
        version: '1',
        id: _.uniqueId('chunked-data-channel_'),
        total: total,
        index: 0,
        data: null
    };
    for (var index = 0; index < total; index++) {
        chunk.index = index;
        // FIXME: type(data) can be String, Blob, ArrayBuffer or ArrayBufferView
        chunk.data = data.substring(offset, offset + chunk_size);
        offset += chunk_size;
        var chunk_json = JSON.stringify(chunk);
        send.call(dc, chunk_json);
    }
  }

  var chunked_msgs = {};

  function chunkedOnMessage(event) {
    console.info('dc chunked "' + dc.label + '" onmessage');
    var msg = JSON.parse(event.data);
    if (!(msg.id in chunked_msgs)) {
        chunked_msgs[msg.id] = new ros_webrtc.ChunkedMessage({
            id: msg.id,
            total: msg.total
        });
    }
    var chunked_msg = chunked_msgs[msg.id];
    chunked_msg.addChunk(msg.index, msg.data);
    if (chunked_msg.isComplete()) {
        delete chunked_msgs[chunked_msg.id];
        if (onmessage) {
          onmessage({data: chunked_msg.merge()});
        }
    }
  }

  dc.send = chunkedSend;
  dc.onmessage = chunkedOnMessage;
};
