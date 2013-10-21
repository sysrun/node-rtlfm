{
  "targets": [
    {
      "target_name": "rtlsdr",
      "sources": [
        "src/binding.cpp",
        "src/rtlsdr.cpp"
      ],
      "conditions" : [
        [
          'OS!="win"', {
            "libraries" : [
              '-lrtlsdr',
            ],
          }
        ],
        [
          'OS=="win"', {
            "libraries" : [
              '<(module_root_dir)/gyp/lib/librtlsdr.dll.a'
            ]
          }
        ]
      ]
    }
  ]
}
