var Settings = Backbone.Model.extend({

  defaults: {
    signaling_server: {
        url: 'ws://127.0.0.1:9000'
    },
    ice_servers: [{
        url: navigator.mozGetUserMedia ? 'stun:stun.services.mozilla.com' :
             navigator.webkitGetUserMedia ? 'stun:stun.l.google.com:19302' :
             'stun:23.21.150.121'
    }],
  },

  initialize: function (attributes, options) {
    options = options || {};
    _.defaults(options , {
        storage: window.localStorage
    });
    this.storage = options.storage;
  },

  fetch: function () {
    _.each(_.keys(this.attributes), function (key) {
      if (this.storage.getItem(key) !== null) {
        this.set(key, JSON.parse(this.storage.getItem(key)));
      } else if (key in this.defaults){
        this.set(key, this.defaults[key]);
      } else {
        this.unset(key);
      }
    }, this);
  },

  reset: function () {
    _.each(
        this.defaults,
        function (value, key) { this.set(key, value); },
        this
    );
  },

  save: function () {
    _.each(
        this.attributes,
        function (v, k) {
            this.storage.setItem(k, v === null ? null : JSON.stringify(v));
        },
        this
    );
  }

});

var SettingsDialog = Backbone.View.extend({

  initialize: function (options) {
      var that = this;

      options = options || {};
      this.settings = options.settings;

      this.$el.find('#settings-toggle').on('click', function () {
          that.toggle();
      });

      this.$el.find('#settings-defaults').on('click', function () {
          that.settings.reset();
          that.load();
      });

      this.$el.find('#settings-save').on('click', $.proxy(function () {
          this.save();
          this.$el.find('#settings').modal('hide');
          (options.location || window.location).reload();
      }, this));

      this.$el.find('#settings').on('shown.bs.modal', function () {
        that.$el.keyup(function (e) {
          if (e.which == 13) {
            that.$el.find('#settings-save').click();
          }
        });
      });
      this.$el.find('#settings').on('hidden.bs.modal', function () {
        that.$el.off('keyup');
      });
  },

  load: function () {
      this.$el.find('#settings-signaling-server-url').val(this.settings.get('signaling_server').url);
  },

  save: function () {
    this.settings.set('signaling_server', {
        url: this.$el.find('#settings-signaling-server-url').val()
    });
    this.settings.save();
  },

  toggle: function () {
      this.settings.fetch();
      this.load();
      this.$el.find('#settings').modal('toggle');
  }

});

var Router = Backbone.Router.extend({

  routes: {
    ':my_id/call/:peer_id': 'call',
    ':my_id/wait': 'wait',
    ':my_id': 'default',
    '': 'root'
  },

  call: function (my_id, peer_id) {
    if  (signaling) {
      window.location.reload();
      return;
    }
    callPeer(my_id, peer_id);
  },

  wait: function (my_id) {
    if  (signaling) {
      window.location.reload();
      return;
    }
    waitForCall(my_id);
  },

  default: function (my_id) {
    this.navigate(my_id + '/wait', {trigger: true});
  },

  root: function () {
    var my_id = UUIDjs.create().toString().replace(/-/g, '');
    this.navigate(my_id + '/wait', {trigger: true});
  }

});

RemotePlayer = Backbone.View.extend({

  initialize: function() {
    this.listenTo(this.model, 'change:stream', this.render);
  },

  render: function() {
    var stream = this.model.get('stream');
    attachMediaStream(this.$el.find('video').get(0), stream);
  },

  toggleFullscreen: function () {
    screenfull.toggle(this.$el.find('video').get(0));
  }

});

SendMessage = Backbone.View.extend({

  defaults: {
    flash_duration: 200
  },

  initialize: function (options) {
    var that = this;

    this.options = _.defaults(options, this.defaults);

    this.$el.find('input').keyup(function (e) {
      switch (e.which) {
        case 13: // enter
          that.model.publish({data: this.value});
          this.value = '';
          that.flash();
          break;

        case 27: // esc
          this.value = '';
          if (document.activeElement != $('body').get(0)) {
            document.activeElement.blur();
          }
          break;
        }
    });
  },

  flash: function () {
    var that = this,
        el = this.$el.find('.msg-flash');

    if (el.is(':animated'))
      return;
    el.animate({opacity: 1.0}, this.options.flash_duration, function () {
      el.animate({opacity: 0.2}, that.options.flash_duration);
    });
  },

});

ReceiveMessages = Backbone.View.extend({

  defaults: {
    flash_duration: 200,
    scroll_speed: 0,
  },

  initialize: function (options) {
    var that = this;

    this.options = _.defaults(options, this.defaults);

    // model

    this.model.subscribe(function (message) {
      that.$el.find('#msg-recv-log ol').append(
        '<li>' + (message.data || '&nbsp;') + '</li>'
      );
      if (that.isScrollActive()) {
        that.scroll(that.options.scroll_speed);
      }
      that.flash();
    });

    // clear

    this.$el.find('#msg-recv-clear').click(function () {
      that.$el.find('#msg-recv-log ol li').remove();
    });

    // resize

    $(window).resize(function () {
      that.resize();
    });
    $('#player video').resize(function () {
      that.resize();
    });
    this.resize();

    // scroll

    this.$el.find('#msg-recv-scroll').click(function () {
      if (!that.isScrollActive()) {
        that.scroll(that.options.scroll_speed);
      }
    });
    if (this.isScrollActive()) {
      this.scroll(this.options.scroll_speed);
    }
  },

  resize: function () {
    var height = ($(window).height() - $('#msg-recv-log').position().top) + "px";
    $('#msg-recv-log').css('height', height);
  },

  flash: function () {
    var that = this,
        el = this.$el.find('.msg-flash');

    if (el.is(':animated'))
      return;
    el.animate({opacity: 1.0}, this.options.flash_duration, function () {
      el.animate({opacity: 0.2}, that.options.flash_duration);
    });
  },

  isScrollActive: function () {
    return this.$el.find('#msg-recv-scroll').hasClass('active');
  },

  scroll: function (duration) {
    var log = this.$el.find('#msg-recv-log');
    duration = _.isUndefined(duration) ? 0 : duration;
    log.animate({ scrollTop: log[0].scrollHeight}, duration);
  }

});

// state

var signaling = null,
    call = null,
    remote_player = null,
    ros = null,
    chatter_topic = null,
    send_msg = null,
    recv_msgs = null;

function initialize(dc) {
  ros = new ROSLIB.Ros();

  ros.transportLibrary = call.pc;
  ros.transportOptions = {
    reliable: false,
    ordered: false,
    protocol: 'application/vnd.rosbridge.v1+json; chunksize=512'
  };
  ros.connect(dc);
  call.adaptROSClient(ros);

  chatter_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/ros_webrtc/chatter',
    messageType: 'std_msgs/String'
  });

  send_msg = new SendMessage({
    el: $('#msg-send').get(0),
    model: chatter_topic
  });

  recv_msgs = new ReceiveMessages({
    el: $('#msg-recv').get(0),
    model: chatter_topic
  });
}

function waitForCall(my_id) {
  signaling = new ros_webrtc.Signaling({
    server: settings.get('signaling_server').url,
    id: my_id
  });

  signaling.once('call', function (signaling, session_id, peer_id) {
    if (call) {
      console.loginfo('busy, ignoring call "' + session_id + '" from "' + peer_id + '"');
      return;
    }
    call = new ros_webrtc.Call({
      id: session_id,
      peer_id: peer_id,
      signaling: signaling
    });
    remote_player = new RemotePlayer({
      el: $('#player'),
      model: call
    });
    call.on('datachannel', function (data_channel) {
      initialize(data_channel);
    });
    call.pickup();
  });
}

function callPeer(my_id, peer_id) {
  signaling = new ros_webrtc.Signaling({
    server: settings.get('signaling_server').url,
    id: my_id
  });

  signaling.once('connected', function () {
    call = new ros_webrtc.Call({
      // id: UUIDjs.create().toString().replace(/-/g, ''),
      id: UUIDjs.create().toString().replace(/-/g, ''),
      peer_id: peer_id,
      signaling: signaling
    });
    remote_player = new RemotePlayer({
      el: $('#player'),
      model: call
    });
    initialize('rosbridge');
    call.dial();
  });
}

// global

var settings = new Settings();

settings.fetch();

var settings_dialog = new SettingsDialog({
    el: $('body').get(0),
    settings: settings
});

$(window).on('beforeunload', function() {
  if (call) {
    call.hangup();
  }
});

$(document).keyup(function (event) {
  var targets = [$('body').get(0) ];
  if (!_.any (targets, function (el) { return event.target ==el; }))
      return;
  switch (event.which) {
      case 32: // space
          send_msg.$el.find('input').focus();
          break;

      case 115: // s
      case 83: // S
          settings_dialog.toggle();
          break;

      case 102: // f
      case 70: // F
          if (remote_player)
            remote_player.toggleFullscreen();
          break;
  }
});

var router = new Router();

Backbone.history.start({ root: window.location.pathname });
