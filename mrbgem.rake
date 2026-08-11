MRuby::Gem::Specification.new('picoruby-pono_tsd') do |spec|
  spec.license = 'MIT'
  spec.author  = 'Yuji Teshima'
  spec.summary = 'PONO TSD series (TSD10/TSD20/TSD50) single-point ToF LiDAR'

  spec.add_dependency 'picoruby-uart'
end
