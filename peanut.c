<?xml version="1.0" encoding="utf-8"?>
<preset>
  <name>Peanut_Logger</name>
  <prepare>
    <convType>0</convType>
    <monoType>1</monoType>
    <edge>128</edge>
    <scanMain>2</scanMain>
    <scanSub>1</scanSub>
    <inverse>1</inverse>
    <bandScanning>0</bandScanning>
    <bandWidth>1</bandWidth>
    <useCustomScript>0</useCustomScript>
    <customScript><![CDATA[]]></customScript>
    <customPreprocessScript><![CDATA[]]></customPreprocessScript>
  </prepare>
  <matrix>
    <maskUsed>00000001</maskUsed>
    <maskAnd>ffffffff</maskAnd>
    <maskOr>00000000</maskOr>
    <maskFill>000000ff</maskFill>
    <operations count="2">
      <operation index="0">
        <mask>ff000000</mask>
        <shift>0</shift>
        <left>0</left>
      </operation>
      <operation index="1">
        <mask>00000001</mask>
        <shift>0</shift>
        <left>0</left>
      </operation>
    </operations>
  </matrix>
  <reordering>
    <operations count="0"/>
  </reordering>
  <image>
    <bytesOrder>0</bytesOrder>
    <blockSize>0</blockSize>
    <blockDefaultOnes>0</blockDefaultOnes>
    <splitToRows>1</splitToRows>
    <compressionRle>0</compressionRle>
    <compressionRleMinLength>2</compressionRleMinLength>
    <blockPrefix><![CDATA[0x]]></blockPrefix>
    <blockSuffix><![CDATA[empty]]></blockSuffix>
    <blockDelimiter><![CDATA[, ]]></blockDelimiter>
    <numeralSystem>16</numeralSystem>
    <previewPrefix><![CDATA[// ]]></previewPrefix>
    <previewSuffix><![CDATA[empty]]></previewSuffix>
    <previewDelimiter><![CDATA[empty]]></previewDelimiter>
    <previewLevels><![CDATA[∙
░
▒
▓
█]]></previewLevels>
  </image>
  <font>
    <bom>0</bom>
    <sortOrder>1</sortOrder>
    <codec>UTF-8</codec>
    <skipMissingCharacters>0</skipMissingCharacters>
    <escapeChars>@"'</escapeChars>
    <escapePrefix>\</escapePrefix>
    <escapeSuffix></escapeSuffix>
  </font>
  <templates>
    <images>:/templates/image_convert</images>
    <fonts>:/templates/font_convert</fonts>
  </templates>
</preset>
