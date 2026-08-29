MRuby::Gem::Specification.new('picoruby-pono_tsd') do |spec|
  spec.license = 'MIT'
  spec.author  = 'Yuji Teshima'
  spec.summary = 'PONO TSD series (TSD10/TSD20/TSD50) single-point ToF LiDAR'

  # picoruby-uart only exists in the PicoRuby tree; a plain mruby build
  # would try to resolve the name against mgem-list and fail. The driver
  # still works there -- construct it with any object that has #read/#write.
  spec.add_dependency 'picoruby-uart' if build.respond_to?(:picoruby?)
end
