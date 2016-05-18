module.exports = function (grunt) {

  grunt.loadNpmTasks('grunt-contrib-connect');
  grunt.loadNpmTasks('grunt-contrib-clean');
  grunt.loadNpmTasks('grunt-contrib-copy');
  grunt.loadNpmTasks('grunt-contrib-jshint');
  grunt.loadNpmTasks('grunt-contrib-watch');

  grunt.config.init({
    pkg: grunt.file.readJSON('package.json'),

    clean: ['dist'],

    connect: {
      server: {
        options: {
          hostname: '*',
          port: 8080,
          debug: false,
          base: 'dist'
        }
      }
    },

    copy: {
      src: {
        files: [{
            cwd: 'src/',
            src: ['*.css'],
            dest: 'dist/css/',
            expand: true
          }, {
            cwd: 'src/',
            src: ['*.js'],
            dest: 'dist/js/',
            expand: true
          }, {
            cwd: 'src/',
            src: ['*.html'],
            dest: 'dist/',
            expand: true
          }]
      },
      vendor: {
        files: [{
            src: 'bower_components/adapterjs/publish/adapter.debug.js',
            dest: 'dist/js/adapter.js'
          }, {
            src: 'bower_components/jquery/jquery.js',
            dest: 'dist/js/jquery.js'
          }, {
            src: 'bower_components/livereload-js/dist/livereload.js',
            dest: 'dist/livereload.js'
          }, {
            src: 'bower_components/bootstrap/dist/js/bootstrap.js',
            dest: 'dist/js/bootstrap.js'
          }, {
              cwd: 'bower_components/bootstrap/dist/css/',
              src: ['bootstrap.css*'],
              dest: 'dist/css/',
              expand: true
          }, {
              cwd: 'bower_components/font-awesome/css/',
              src: ['*'],
              dest: 'dist/css',
              expand: true
          }, {
            cwd: 'bower_components/font-awesome/fonts/',
            src: ['*'],
            dest: 'dist/fonts',
            expand: true
          }, {
            src: 'bower_components/screenfull/dist/screenfull.js',
            dest: 'dist/js/screenfull.js'
          }, {
              src: 'bower_components/jquery/dist/jquery.js',
              dest: 'dist/js/jquery.js'
          }, {
            src: 'bower_components/backbone/backbone.js',
            dest: 'dist/js/backbone.js'
          }, {
            src: 'bower_components/underscore/underscore.js',
            dest: 'dist/js/underscore.js'
          }, {
            src: 'bower_components/uuid-js/lib/uuid.js',
            dest: 'dist/js/uuid.js'
          }, {
            src: 'bower_components/meatyjs/dist/meaty.js',
            dest: 'dist/js/meaty.js'
          }, {
            src: 'bower_components/roslib/build/roslib.js',
            dest: 'dist/js/roslib.js'
          }, {
            src: 'bower_components/eventemitter2/lib/eventemitter2.js',
            dest: 'dist/js/eventemitter2.js'
          }, {
            src: 'bower_components/screenfull/dist/screenfull.js',
            dest: 'dist/js/screenfull.js'
          }]
      }
    },

    jshint: {
      grunt: [
        'gruntfile.js'
      ],
      src: [
        'gruntfile.js',
        'src/*.js'
      ],
      options: {
        debug: true
      }
    },

    watch: {
      grunt: {
        files: 'Gruntfile.js'
      },
      src: {
        options: {
          livereload: true
        },
        files: [
            'bower_components/**',
            'src/**'
         ],
        tasks: ['build']
      }
    }

  });

  grunt.registerTask('build', ['jshint', 'copy:vendor', 'copy:src']);
  grunt.registerTask('dev', ['clean', 'build', 'connect', 'watch']);
  grunt.registerTask('default', ['build']);
};
