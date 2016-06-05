<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="7.5.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="con-wago255">
<description>&lt;b&gt;Wago Connectors&lt;/b&gt;&lt;p&gt;
Types 233 and 234&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="233-224">
<description>&lt;b&gt;WAGO&lt;/b&gt;</description>
<wire x1="-31" y1="4.8" x2="-31" y2="4.25" width="0.2032" layer="21"/>
<wire x1="-31" y1="4.25" x2="-31.35" y2="4.25" width="0.2032" layer="21"/>
<wire x1="-31.35" y1="4.25" x2="-31.35" y2="3.25" width="0.2032" layer="21"/>
<wire x1="-31.35" y1="3.25" x2="-31" y2="3.25" width="0.2032" layer="21"/>
<wire x1="-31" y1="3.25" x2="-31" y2="-1.25" width="0.2032" layer="21"/>
<wire x1="-31" y1="-1.25" x2="-31.35" y2="-1.25" width="0.2032" layer="21"/>
<wire x1="-31.35" y1="-1.25" x2="-31.35" y2="-3.25" width="0.2032" layer="21"/>
<wire x1="-31.35" y1="-3.25" x2="-31" y2="-3.25" width="0.2032" layer="21"/>
<wire x1="-31" y1="-3.25" x2="-31" y2="-5.25" width="0.2032" layer="21"/>
<wire x1="-31" y1="-5.25" x2="-31.35" y2="-5.25" width="0.2032" layer="21"/>
<wire x1="-31.35" y1="-5.25" x2="-31.35" y2="-6.25" width="0.2032" layer="21"/>
<wire x1="-31.35" y1="-6.25" x2="-31" y2="-6.25" width="0.2032" layer="21"/>
<wire x1="-31" y1="-6.25" x2="-31" y2="-7.1" width="0.2032" layer="21"/>
<wire x1="-31" y1="-7.1" x2="30.75" y2="-7.1" width="0.2032" layer="21"/>
<wire x1="30.75" y1="-7.1" x2="30.75" y2="-6.25" width="0.2032" layer="21"/>
<wire x1="30.75" y1="-5.25" x2="30.75" y2="-3.75" width="0.2032" layer="21"/>
<wire x1="30.75" y1="-3.75" x2="30.75" y2="-3.25" width="0.2032" layer="21"/>
<wire x1="30.75" y1="-1.25" x2="30.75" y2="3.25" width="0.2032" layer="21"/>
<wire x1="30.75" y1="4.25" x2="30.75" y2="4.8" width="0.2032" layer="21"/>
<wire x1="30.75" y1="4.8" x2="-31" y2="4.8" width="0.2032" layer="21"/>
<wire x1="-30" y1="4.75" x2="-30" y2="-7" width="0.2032" layer="21"/>
<wire x1="-29.5" y1="3.75" x2="-29.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-29.5" y1="1.25" x2="-29.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-29.25" y1="1.25" x2="-28.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-28.25" y1="1.25" x2="-28" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-28" y1="1.25" x2="-28" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-28" y1="3.75" x2="-29.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-31" y1="-3.75" x2="30.75" y2="-3.75" width="0.2032" layer="21"/>
<wire x1="-27.5" y1="4.75" x2="-27.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="-25" y1="4.75" x2="-25" y2="-7" width="0.2032" layer="21"/>
<wire x1="-22.5" y1="4.75" x2="-22.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="-20" y1="4.75" x2="-20" y2="-7" width="0.2032" layer="21"/>
<wire x1="-17.5" y1="4.75" x2="-17.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="-15" y1="4.75" x2="-15" y2="-7" width="0.2032" layer="21"/>
<wire x1="-12.5" y1="4.75" x2="-12.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="-10" y1="4.75" x2="-10" y2="-7" width="0.2032" layer="21"/>
<wire x1="-7.5" y1="4.75" x2="-7.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="-5" y1="4.75" x2="-5" y2="-7" width="0.2032" layer="21"/>
<wire x1="-2.5" y1="4.75" x2="-2.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="0" y1="4.75" x2="0" y2="-7" width="0.2032" layer="21"/>
<wire x1="2.5" y1="4.75" x2="2.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="5" y1="4.75" x2="5" y2="-7" width="0.2032" layer="21"/>
<wire x1="7.5" y1="4.75" x2="7.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="10" y1="4.75" x2="10" y2="-7" width="0.2032" layer="21"/>
<wire x1="12.5" y1="4.75" x2="12.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="15" y1="4.75" x2="15" y2="-7" width="0.2032" layer="21"/>
<wire x1="17.5" y1="4.75" x2="17.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="20" y1="4.75" x2="20" y2="-7" width="0.2032" layer="21"/>
<wire x1="22.5" y1="4.75" x2="22.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="25" y1="4.75" x2="25" y2="-7" width="0.2032" layer="21"/>
<wire x1="27.5" y1="4.75" x2="27.5" y2="-7" width="0.2032" layer="21"/>
<wire x1="30" y1="4.75" x2="30" y2="-7" width="0.2032" layer="21"/>
<wire x1="-29.25" y1="-4.25" x2="-29.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-29.25" y1="-5.75" x2="-28.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-28.25" y1="-5.75" x2="-28.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-28.25" y1="-4.25" x2="-29.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="30.75" y1="4.25" x2="30.4" y2="4.25" width="0.2032" layer="21"/>
<wire x1="30.4" y1="4.25" x2="30.4" y2="3.25" width="0.2032" layer="21"/>
<wire x1="30.4" y1="3.25" x2="30.75" y2="3.25" width="0.2032" layer="21"/>
<wire x1="30.75" y1="-1.25" x2="30.4" y2="-1.25" width="0.2032" layer="21"/>
<wire x1="30.4" y1="-1.25" x2="30.4" y2="-3.25" width="0.2032" layer="21"/>
<wire x1="30.4" y1="-3.25" x2="30.75" y2="-3.25" width="0.2032" layer="21"/>
<wire x1="30.75" y1="-5.25" x2="30.4" y2="-5.25" width="0.2032" layer="21"/>
<wire x1="30.4" y1="-5.25" x2="30.4" y2="-6.25" width="0.2032" layer="21"/>
<wire x1="30.4" y1="-6.25" x2="30.75" y2="-6.25" width="0.2032" layer="21"/>
<wire x1="-26.75" y1="-4.25" x2="-26.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-26.75" y1="-5.75" x2="-25.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-25.75" y1="-5.75" x2="-25.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-25.75" y1="-4.25" x2="-26.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-24.25" y1="-4.25" x2="-24.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-24.25" y1="-5.75" x2="-23.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-23.25" y1="-5.75" x2="-23.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-23.25" y1="-4.25" x2="-24.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-21.75" y1="-4.25" x2="-21.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-21.75" y1="-5.75" x2="-20.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-20.75" y1="-5.75" x2="-20.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-20.75" y1="-4.25" x2="-21.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-19.25" y1="-4.25" x2="-19.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-19.25" y1="-5.75" x2="-18.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-18.25" y1="-5.75" x2="-18.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-18.25" y1="-4.25" x2="-19.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-16.75" y1="-4.25" x2="-16.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-16.75" y1="-5.75" x2="-15.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-15.75" y1="-5.75" x2="-15.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-15.75" y1="-4.25" x2="-16.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-14.25" y1="-4.25" x2="-14.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-14.25" y1="-5.75" x2="-13.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-13.25" y1="-5.75" x2="-13.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-13.25" y1="-4.25" x2="-14.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-11.75" y1="-4.25" x2="-11.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-11.75" y1="-5.75" x2="-10.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-10.75" y1="-5.75" x2="-10.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-10.75" y1="-4.25" x2="-11.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-9.25" y1="-4.25" x2="-9.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-9.25" y1="-5.75" x2="-8.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-8.25" y1="-5.75" x2="-8.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-8.25" y1="-4.25" x2="-9.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-6.75" y1="-4.25" x2="-6.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-6.75" y1="-5.75" x2="-5.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-5.75" y1="-5.75" x2="-5.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-5.75" y1="-4.25" x2="-6.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-4.25" y1="-4.25" x2="-4.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-4.25" y1="-5.75" x2="-3.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-3.25" y1="-5.75" x2="-3.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-3.25" y1="-4.25" x2="-4.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-1.75" y1="-4.25" x2="-1.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-1.75" y1="-5.75" x2="-0.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="-0.75" y1="-5.75" x2="-0.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-0.75" y1="-4.25" x2="-1.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="0.75" y1="-4.25" x2="0.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="0.75" y1="-5.75" x2="1.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="1.75" y1="-5.75" x2="1.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="1.75" y1="-4.25" x2="0.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="3.25" y1="-4.25" x2="3.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="3.25" y1="-5.75" x2="4.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="4.25" y1="-5.75" x2="4.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="4.25" y1="-4.25" x2="3.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="5.75" y1="-4.25" x2="5.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="5.75" y1="-5.75" x2="6.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="6.75" y1="-5.75" x2="6.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="6.75" y1="-4.25" x2="5.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="8.25" y1="-4.25" x2="8.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="8.25" y1="-5.75" x2="9.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="9.25" y1="-5.75" x2="9.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="9.25" y1="-4.25" x2="8.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="10.75" y1="-4.25" x2="10.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="10.75" y1="-5.75" x2="11.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="11.75" y1="-5.75" x2="11.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="11.75" y1="-4.25" x2="10.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="13.25" y1="-4.25" x2="13.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="13.25" y1="-5.75" x2="14.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="14.25" y1="-5.75" x2="14.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="14.25" y1="-4.25" x2="13.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="15.75" y1="-4.25" x2="15.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="15.75" y1="-5.75" x2="16.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="16.75" y1="-5.75" x2="16.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="16.75" y1="-4.25" x2="15.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="18.25" y1="-4.25" x2="18.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="18.25" y1="-5.75" x2="19.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="19.25" y1="-5.75" x2="19.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="19.25" y1="-4.25" x2="18.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="20.75" y1="-4.25" x2="20.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="20.75" y1="-5.75" x2="21.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="21.75" y1="-5.75" x2="21.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="21.75" y1="-4.25" x2="20.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="23.25" y1="-4.25" x2="23.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="23.25" y1="-5.75" x2="24.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="24.25" y1="-5.75" x2="24.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="24.25" y1="-4.25" x2="23.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="25.75" y1="-4.25" x2="25.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="25.75" y1="-5.75" x2="26.75" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="26.75" y1="-5.75" x2="26.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="26.75" y1="-4.25" x2="25.75" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="28.25" y1="-4.25" x2="28.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="28.25" y1="-5.75" x2="29.25" y2="-5.75" width="0.2032" layer="21"/>
<wire x1="29.25" y1="-5.75" x2="29.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="29.25" y1="-4.25" x2="28.25" y2="-4.25" width="0.2032" layer="21"/>
<wire x1="-29.25" y1="3.25" x2="-29.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-29.25" y1="1.75" x2="-28.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-28.25" y1="1.75" x2="-28.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-28.25" y1="3.25" x2="-29.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-29.25" y1="1.25" x2="-29.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-29.25" y1="-1" x2="-28.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-28.25" y1="-1" x2="-28.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-27" y1="3.75" x2="-27" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-27" y1="1.25" x2="-26.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-26.75" y1="1.25" x2="-25.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-25.75" y1="1.25" x2="-25.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-25.5" y1="1.25" x2="-25.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-25.5" y1="3.75" x2="-27" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-26.75" y1="3.25" x2="-26.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-26.75" y1="1.75" x2="-25.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-25.75" y1="1.75" x2="-25.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-25.75" y1="3.25" x2="-26.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-26.75" y1="1.25" x2="-26.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-26.75" y1="-1" x2="-25.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-25.75" y1="-1" x2="-25.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-24.5" y1="3.75" x2="-24.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-24.5" y1="1.25" x2="-24.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-24.25" y1="1.25" x2="-23.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-23.25" y1="1.25" x2="-23" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-23" y1="1.25" x2="-23" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-23" y1="3.75" x2="-24.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-24.25" y1="3.25" x2="-24.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-24.25" y1="1.75" x2="-23.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-23.25" y1="1.75" x2="-23.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-23.25" y1="3.25" x2="-24.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-24.25" y1="1.25" x2="-24.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-24.25" y1="-1" x2="-23.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-23.25" y1="-1" x2="-23.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-22" y1="3.75" x2="-22" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-22" y1="1.25" x2="-21.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-21.75" y1="1.25" x2="-20.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-20.75" y1="1.25" x2="-20.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-20.5" y1="1.25" x2="-20.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-20.5" y1="3.75" x2="-22" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-21.75" y1="3.25" x2="-21.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-21.75" y1="1.75" x2="-20.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-20.75" y1="1.75" x2="-20.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-20.75" y1="3.25" x2="-21.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-21.75" y1="1.25" x2="-21.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-21.75" y1="-1" x2="-20.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-20.75" y1="-1" x2="-20.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-19.5" y1="3.75" x2="-19.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-19.5" y1="1.25" x2="-19.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-19.25" y1="1.25" x2="-18.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-18.25" y1="1.25" x2="-18" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-18" y1="1.25" x2="-18" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-18" y1="3.75" x2="-19.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-19.25" y1="3.25" x2="-19.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-19.25" y1="1.75" x2="-18.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-18.25" y1="1.75" x2="-18.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-18.25" y1="3.25" x2="-19.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-19.25" y1="1.25" x2="-19.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-19.25" y1="-1" x2="-18.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-18.25" y1="-1" x2="-18.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-17" y1="3.75" x2="-17" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-17" y1="1.25" x2="-16.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-16.75" y1="1.25" x2="-15.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-15.75" y1="1.25" x2="-15.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-15.5" y1="1.25" x2="-15.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-15.5" y1="3.75" x2="-17" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-16.75" y1="3.25" x2="-16.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-16.75" y1="1.75" x2="-15.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-15.75" y1="1.75" x2="-15.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-15.75" y1="3.25" x2="-16.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-16.75" y1="1.25" x2="-16.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-16.75" y1="-1" x2="-15.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-15.75" y1="-1" x2="-15.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-14.5" y1="3.75" x2="-14.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-14.5" y1="1.25" x2="-14.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-14.25" y1="1.25" x2="-13.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-13.25" y1="1.25" x2="-13" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-13" y1="1.25" x2="-13" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-13" y1="3.75" x2="-14.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-14.25" y1="3.25" x2="-14.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-14.25" y1="1.75" x2="-13.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-13.25" y1="1.75" x2="-13.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-13.25" y1="3.25" x2="-14.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-14.25" y1="1.25" x2="-14.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-14.25" y1="-1" x2="-13.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-13.25" y1="-1" x2="-13.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-12" y1="3.75" x2="-12" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-12" y1="1.25" x2="-11.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-11.75" y1="1.25" x2="-10.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-10.75" y1="1.25" x2="-10.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-10.5" y1="1.25" x2="-10.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-10.5" y1="3.75" x2="-12" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-11.75" y1="3.25" x2="-11.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-11.75" y1="1.75" x2="-10.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-10.75" y1="1.75" x2="-10.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-10.75" y1="3.25" x2="-11.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-11.75" y1="1.25" x2="-11.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-11.75" y1="-1" x2="-10.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-10.75" y1="-1" x2="-10.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-9.5" y1="3.75" x2="-9.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-9.5" y1="1.25" x2="-9.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-9.25" y1="1.25" x2="-8.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-8.25" y1="1.25" x2="-8" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-8" y1="1.25" x2="-8" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-8" y1="3.75" x2="-9.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-9.25" y1="3.25" x2="-9.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-9.25" y1="1.75" x2="-8.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-8.25" y1="1.75" x2="-8.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-8.25" y1="3.25" x2="-9.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-9.25" y1="1.25" x2="-9.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-9.25" y1="-1" x2="-8.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-8.25" y1="-1" x2="-8.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-7" y1="3.75" x2="-7" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-7" y1="1.25" x2="-6.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-6.75" y1="1.25" x2="-5.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-5.75" y1="1.25" x2="-5.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-5.5" y1="1.25" x2="-5.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-5.5" y1="3.75" x2="-7" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-6.75" y1="3.25" x2="-6.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-6.75" y1="1.75" x2="-5.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-5.75" y1="1.75" x2="-5.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-5.75" y1="3.25" x2="-6.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-6.75" y1="1.25" x2="-6.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-6.75" y1="-1" x2="-5.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-5.75" y1="-1" x2="-5.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-4.5" y1="3.75" x2="-4.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-4.5" y1="1.25" x2="-4.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-4.25" y1="1.25" x2="-3.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-3.25" y1="1.25" x2="-3" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-3" y1="1.25" x2="-3" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-3" y1="3.75" x2="-4.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-4.25" y1="3.25" x2="-4.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-4.25" y1="1.75" x2="-3.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-3.25" y1="1.75" x2="-3.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-3.25" y1="3.25" x2="-4.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-4.25" y1="1.25" x2="-4.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-4.25" y1="-1" x2="-3.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="-3.25" y1="-1" x2="-3.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-2" y1="3.75" x2="-2" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-2" y1="1.25" x2="-1.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-1.75" y1="1.25" x2="-0.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-0.75" y1="1.25" x2="-0.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="-0.5" y1="1.25" x2="-0.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-0.5" y1="3.75" x2="-2" y2="3.75" width="0.2032" layer="51"/>
<wire x1="-1.75" y1="3.25" x2="-1.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-1.75" y1="1.75" x2="-0.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="-0.75" y1="1.75" x2="-0.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-0.75" y1="3.25" x2="-1.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="-1.75" y1="1.25" x2="-1.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-1.75" y1="-1" x2="-0.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="-0.75" y1="-1" x2="-0.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="0.5" y1="3.75" x2="0.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="0.5" y1="1.25" x2="0.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="0.75" y1="1.25" x2="1.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="1.75" y1="1.25" x2="2" y2="1.25" width="0.2032" layer="51"/>
<wire x1="2" y1="1.25" x2="2" y2="3.75" width="0.2032" layer="51"/>
<wire x1="2" y1="3.75" x2="0.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="0.75" y1="3.25" x2="0.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="0.75" y1="1.75" x2="1.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="1.75" y1="1.75" x2="1.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="1.75" y1="3.25" x2="0.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="0.75" y1="1.25" x2="0.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="0.75" y1="-1" x2="1.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="1.75" y1="-1" x2="1.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="3" y1="3.75" x2="3" y2="1.25" width="0.2032" layer="51"/>
<wire x1="3" y1="1.25" x2="3.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="3.25" y1="1.25" x2="4.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="4.25" y1="1.25" x2="4.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="4.5" y1="1.25" x2="4.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="4.5" y1="3.75" x2="3" y2="3.75" width="0.2032" layer="51"/>
<wire x1="3.25" y1="3.25" x2="3.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="3.25" y1="1.75" x2="4.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="4.25" y1="1.75" x2="4.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="4.25" y1="3.25" x2="3.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="3.25" y1="1.25" x2="3.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="3.25" y1="-1" x2="4.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="4.25" y1="-1" x2="4.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="5.5" y1="3.75" x2="5.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="5.5" y1="1.25" x2="5.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="5.75" y1="1.25" x2="6.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="6.75" y1="1.25" x2="7" y2="1.25" width="0.2032" layer="51"/>
<wire x1="7" y1="1.25" x2="7" y2="3.75" width="0.2032" layer="51"/>
<wire x1="7" y1="3.75" x2="5.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="5.75" y1="3.25" x2="5.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="5.75" y1="1.75" x2="6.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="6.75" y1="1.75" x2="6.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="6.75" y1="3.25" x2="5.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="5.75" y1="1.25" x2="5.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="5.75" y1="-1" x2="6.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="6.75" y1="-1" x2="6.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="8" y1="3.75" x2="8" y2="1.25" width="0.2032" layer="51"/>
<wire x1="8" y1="1.25" x2="8.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="8.25" y1="1.25" x2="9.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="9.25" y1="1.25" x2="9.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="9.5" y1="1.25" x2="9.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="9.5" y1="3.75" x2="8" y2="3.75" width="0.2032" layer="51"/>
<wire x1="8.25" y1="3.25" x2="8.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="8.25" y1="1.75" x2="9.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="9.25" y1="1.75" x2="9.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="9.25" y1="3.25" x2="8.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="8.25" y1="1.25" x2="8.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="8.25" y1="-1" x2="9.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="9.25" y1="-1" x2="9.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="10.5" y1="3.75" x2="10.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="10.5" y1="1.25" x2="10.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="10.75" y1="1.25" x2="11.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="11.75" y1="1.25" x2="12" y2="1.25" width="0.2032" layer="51"/>
<wire x1="12" y1="1.25" x2="12" y2="3.75" width="0.2032" layer="51"/>
<wire x1="12" y1="3.75" x2="10.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="10.75" y1="3.25" x2="10.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="10.75" y1="1.75" x2="11.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="11.75" y1="1.75" x2="11.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="11.75" y1="3.25" x2="10.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="10.75" y1="1.25" x2="10.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="10.75" y1="-1" x2="11.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="11.75" y1="-1" x2="11.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="13" y1="3.75" x2="13" y2="1.25" width="0.2032" layer="51"/>
<wire x1="13" y1="1.25" x2="13.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="13.25" y1="1.25" x2="14.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="14.25" y1="1.25" x2="14.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="14.5" y1="1.25" x2="14.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="14.5" y1="3.75" x2="13" y2="3.75" width="0.2032" layer="51"/>
<wire x1="13.25" y1="3.25" x2="13.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="13.25" y1="1.75" x2="14.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="14.25" y1="1.75" x2="14.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="14.25" y1="3.25" x2="13.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="13.25" y1="1.25" x2="13.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="13.25" y1="-1" x2="14.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="14.25" y1="-1" x2="14.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="15.5" y1="3.75" x2="15.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="15.5" y1="1.25" x2="15.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="15.75" y1="1.25" x2="16.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="16.75" y1="1.25" x2="17" y2="1.25" width="0.2032" layer="51"/>
<wire x1="17" y1="1.25" x2="17" y2="3.75" width="0.2032" layer="51"/>
<wire x1="17" y1="3.75" x2="15.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="15.75" y1="3.25" x2="15.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="15.75" y1="1.75" x2="16.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="16.75" y1="1.75" x2="16.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="16.75" y1="3.25" x2="15.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="15.75" y1="1.25" x2="15.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="15.75" y1="-1" x2="16.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="16.75" y1="-1" x2="16.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="18" y1="3.75" x2="18" y2="1.25" width="0.2032" layer="51"/>
<wire x1="18" y1="1.25" x2="18.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="18.25" y1="1.25" x2="19.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="19.25" y1="1.25" x2="19.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="19.5" y1="1.25" x2="19.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="19.5" y1="3.75" x2="18" y2="3.75" width="0.2032" layer="51"/>
<wire x1="18.25" y1="3.25" x2="18.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="18.25" y1="1.75" x2="19.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="19.25" y1="1.75" x2="19.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="19.25" y1="3.25" x2="18.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="18.25" y1="1.25" x2="18.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="18.25" y1="-1" x2="19.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="19.25" y1="-1" x2="19.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="20.5" y1="3.75" x2="20.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="20.5" y1="1.25" x2="20.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="20.75" y1="1.25" x2="21.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="21.75" y1="1.25" x2="22" y2="1.25" width="0.2032" layer="51"/>
<wire x1="22" y1="1.25" x2="22" y2="3.75" width="0.2032" layer="51"/>
<wire x1="22" y1="3.75" x2="20.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="20.75" y1="3.25" x2="20.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="20.75" y1="1.75" x2="21.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="21.75" y1="1.75" x2="21.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="21.75" y1="3.25" x2="20.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="20.75" y1="1.25" x2="20.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="20.75" y1="-1" x2="21.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="21.75" y1="-1" x2="21.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="23" y1="3.75" x2="23" y2="1.25" width="0.2032" layer="51"/>
<wire x1="23" y1="1.25" x2="23.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="23.25" y1="1.25" x2="24.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="24.25" y1="1.25" x2="24.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="24.5" y1="1.25" x2="24.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="24.5" y1="3.75" x2="23" y2="3.75" width="0.2032" layer="51"/>
<wire x1="23.25" y1="3.25" x2="23.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="23.25" y1="1.75" x2="24.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="24.25" y1="1.75" x2="24.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="24.25" y1="3.25" x2="23.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="23.25" y1="1.25" x2="23.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="23.25" y1="-1" x2="24.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="24.25" y1="-1" x2="24.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="25.5" y1="3.75" x2="25.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="25.5" y1="1.25" x2="25.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="25.75" y1="1.25" x2="26.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="26.75" y1="1.25" x2="27" y2="1.25" width="0.2032" layer="51"/>
<wire x1="27" y1="1.25" x2="27" y2="3.75" width="0.2032" layer="51"/>
<wire x1="27" y1="3.75" x2="25.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="25.75" y1="3.25" x2="25.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="25.75" y1="1.75" x2="26.75" y2="1.75" width="0.2032" layer="51"/>
<wire x1="26.75" y1="1.75" x2="26.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="26.75" y1="3.25" x2="25.75" y2="3.25" width="0.2032" layer="51"/>
<wire x1="25.75" y1="1.25" x2="25.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="25.75" y1="-1" x2="26.75" y2="-1" width="0.2032" layer="51"/>
<wire x1="26.75" y1="-1" x2="26.75" y2="1.25" width="0.2032" layer="51"/>
<wire x1="28" y1="3.75" x2="28" y2="1.25" width="0.2032" layer="51"/>
<wire x1="28" y1="1.25" x2="28.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="28.25" y1="1.25" x2="29.25" y2="1.25" width="0.2032" layer="51"/>
<wire x1="29.25" y1="1.25" x2="29.5" y2="1.25" width="0.2032" layer="51"/>
<wire x1="29.5" y1="1.25" x2="29.5" y2="3.75" width="0.2032" layer="51"/>
<wire x1="29.5" y1="3.75" x2="28" y2="3.75" width="0.2032" layer="51"/>
<wire x1="28.25" y1="3.25" x2="28.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="28.25" y1="1.75" x2="29.25" y2="1.75" width="0.2032" layer="51"/>
<wire x1="29.25" y1="1.75" x2="29.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="29.25" y1="3.25" x2="28.25" y2="3.25" width="0.2032" layer="51"/>
<wire x1="28.25" y1="1.25" x2="28.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="28.25" y1="-1" x2="29.25" y2="-1" width="0.2032" layer="51"/>
<wire x1="29.25" y1="-1" x2="29.25" y2="1.25" width="0.2032" layer="51"/>
<pad name="A1" x="-28.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B1" x="-28.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A2" x="-26.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B2" x="-26.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A3" x="-23.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B3" x="-23.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A4" x="-21.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B4" x="-21.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A5" x="-18.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B5" x="-18.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A6" x="-16.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B6" x="-16.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A7" x="-13.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B7" x="-13.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A8" x="-11.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B8" x="-11.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A9" x="-8.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B9" x="-8.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A10" x="-6.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B10" x="-6.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A11" x="-3.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B11" x="-3.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A12" x="-1.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B12" x="-1.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A13" x="1.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B13" x="1.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A14" x="3.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B14" x="3.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A15" x="6.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B15" x="6.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A16" x="8.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B16" x="8.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A17" x="11.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B17" x="11.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A18" x="13.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B18" x="13.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A19" x="16.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B19" x="16.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A20" x="18.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B20" x="18.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A21" x="21.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B21" x="21.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A22" x="23.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B22" x="23.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A23" x="26.25" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B23" x="26.25" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="A24" x="28.75" y="2.5" drill="1.2" shape="long" rot="R90"/>
<pad name="B24" x="28.75" y="-2.5" drill="1.2" shape="long" rot="R90"/>
<text x="-28.85" y="5.25" size="1.27" layer="25">&gt;NAME</text>
<text x="-19.6" y="5.25" size="1.27" layer="27">&gt;VALUE</text>
<text x="-28.9" y="-6.85" size="0.8128" layer="21">1</text>
<text x="-6.9" y="-6.85" size="0.8128" layer="21">10</text>
<text x="18.1" y="-6.85" size="0.8128" layer="21">20</text>
<text x="-19.15" y="-6.85" size="0.8128" layer="21">5</text>
<text x="5.6" y="-6.85" size="0.8128" layer="21">15</text>
</package>
</packages>
<symbols>
<symbol name="KL-24">
<wire x1="-2.54" y1="28.194" x2="-2.54" y2="27.686" width="0.254" layer="94"/>
<wire x1="-2.54" y1="27.686" x2="-1.016" y2="27.686" width="0.254" layer="94"/>
<wire x1="-1.016" y1="27.686" x2="-1.016" y2="28.194" width="0.254" layer="94"/>
<wire x1="-1.016" y1="28.194" x2="-2.54" y2="28.194" width="0.254" layer="94"/>
<wire x1="-2.54" y1="25.654" x2="-2.54" y2="25.146" width="0.254" layer="94"/>
<wire x1="-2.54" y1="25.146" x2="-1.016" y2="25.146" width="0.254" layer="94"/>
<wire x1="-1.016" y1="25.146" x2="-1.016" y2="25.654" width="0.254" layer="94"/>
<wire x1="-1.016" y1="25.654" x2="-2.54" y2="25.654" width="0.254" layer="94"/>
<wire x1="-2.54" y1="23.114" x2="-2.54" y2="22.606" width="0.254" layer="94"/>
<wire x1="-2.54" y1="22.606" x2="-1.016" y2="22.606" width="0.254" layer="94"/>
<wire x1="-1.016" y1="22.606" x2="-1.016" y2="23.114" width="0.254" layer="94"/>
<wire x1="-1.016" y1="23.114" x2="-2.54" y2="23.114" width="0.254" layer="94"/>
<wire x1="-2.54" y1="20.574" x2="-2.54" y2="20.066" width="0.254" layer="94"/>
<wire x1="-2.54" y1="20.066" x2="-1.016" y2="20.066" width="0.254" layer="94"/>
<wire x1="-1.016" y1="20.066" x2="-1.016" y2="20.574" width="0.254" layer="94"/>
<wire x1="-1.016" y1="20.574" x2="-2.54" y2="20.574" width="0.254" layer="94"/>
<wire x1="-2.54" y1="18.034" x2="-2.54" y2="17.526" width="0.254" layer="94"/>
<wire x1="-2.54" y1="17.526" x2="-1.016" y2="17.526" width="0.254" layer="94"/>
<wire x1="-1.016" y1="17.526" x2="-1.016" y2="18.034" width="0.254" layer="94"/>
<wire x1="-1.016" y1="18.034" x2="-2.54" y2="18.034" width="0.254" layer="94"/>
<wire x1="-2.54" y1="15.494" x2="-2.54" y2="14.986" width="0.254" layer="94"/>
<wire x1="-2.54" y1="14.986" x2="-1.016" y2="14.986" width="0.254" layer="94"/>
<wire x1="-1.016" y1="14.986" x2="-1.016" y2="15.494" width="0.254" layer="94"/>
<wire x1="-1.016" y1="15.494" x2="-2.54" y2="15.494" width="0.254" layer="94"/>
<wire x1="-2.54" y1="12.954" x2="-2.54" y2="12.446" width="0.254" layer="94"/>
<wire x1="-2.54" y1="12.446" x2="-1.016" y2="12.446" width="0.254" layer="94"/>
<wire x1="-1.016" y1="12.446" x2="-1.016" y2="12.954" width="0.254" layer="94"/>
<wire x1="-1.016" y1="12.954" x2="-2.54" y2="12.954" width="0.254" layer="94"/>
<wire x1="-2.54" y1="10.414" x2="-2.54" y2="9.906" width="0.254" layer="94"/>
<wire x1="-2.54" y1="9.906" x2="-1.016" y2="9.906" width="0.254" layer="94"/>
<wire x1="-1.016" y1="9.906" x2="-1.016" y2="10.414" width="0.254" layer="94"/>
<wire x1="-1.016" y1="10.414" x2="-2.54" y2="10.414" width="0.254" layer="94"/>
<wire x1="-2.54" y1="7.874" x2="-2.54" y2="7.366" width="0.254" layer="94"/>
<wire x1="-2.54" y1="7.366" x2="-1.016" y2="7.366" width="0.254" layer="94"/>
<wire x1="-1.016" y1="7.366" x2="-1.016" y2="7.874" width="0.254" layer="94"/>
<wire x1="-1.016" y1="7.874" x2="-2.54" y2="7.874" width="0.254" layer="94"/>
<wire x1="-2.54" y1="5.334" x2="-2.54" y2="4.826" width="0.254" layer="94"/>
<wire x1="-2.54" y1="4.826" x2="-1.016" y2="4.826" width="0.254" layer="94"/>
<wire x1="-1.016" y1="4.826" x2="-1.016" y2="5.334" width="0.254" layer="94"/>
<wire x1="-1.016" y1="5.334" x2="-2.54" y2="5.334" width="0.254" layer="94"/>
<wire x1="-2.54" y1="2.794" x2="-2.54" y2="2.286" width="0.254" layer="94"/>
<wire x1="-2.54" y1="2.286" x2="-1.016" y2="2.286" width="0.254" layer="94"/>
<wire x1="-1.016" y1="2.286" x2="-1.016" y2="2.794" width="0.254" layer="94"/>
<wire x1="-1.016" y1="2.794" x2="-2.54" y2="2.794" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0.254" x2="-2.54" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-0.254" x2="-1.016" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-0.254" x2="-1.016" y2="0.254" width="0.254" layer="94"/>
<wire x1="-1.016" y1="0.254" x2="-2.54" y2="0.254" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-2.286" x2="-2.54" y2="-2.794" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-2.794" x2="-1.016" y2="-2.794" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-2.794" x2="-1.016" y2="-2.286" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-2.286" x2="-2.54" y2="-2.286" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-4.826" x2="-2.54" y2="-5.334" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-5.334" x2="-1.016" y2="-5.334" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-5.334" x2="-1.016" y2="-4.826" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-4.826" x2="-2.54" y2="-4.826" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-7.366" x2="-2.54" y2="-7.874" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-7.874" x2="-1.016" y2="-7.874" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-7.874" x2="-1.016" y2="-7.366" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-7.366" x2="-2.54" y2="-7.366" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-9.906" x2="-2.54" y2="-10.414" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-10.414" x2="-1.016" y2="-10.414" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-10.414" x2="-1.016" y2="-9.906" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-9.906" x2="-2.54" y2="-9.906" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-12.446" x2="-2.54" y2="-12.954" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-12.954" x2="-1.016" y2="-12.954" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-12.954" x2="-1.016" y2="-12.446" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-12.446" x2="-2.54" y2="-12.446" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-14.986" x2="-2.54" y2="-15.494" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-15.494" x2="-1.016" y2="-15.494" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-15.494" x2="-1.016" y2="-14.986" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-14.986" x2="-2.54" y2="-14.986" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-17.526" x2="-2.54" y2="-18.034" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-18.034" x2="-1.016" y2="-18.034" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-18.034" x2="-1.016" y2="-17.526" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-17.526" x2="-2.54" y2="-17.526" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-20.066" x2="-2.54" y2="-20.574" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-20.574" x2="-1.016" y2="-20.574" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-20.574" x2="-1.016" y2="-20.066" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-20.066" x2="-2.54" y2="-20.066" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-22.606" x2="-2.54" y2="-23.114" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-23.114" x2="-1.016" y2="-23.114" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-23.114" x2="-1.016" y2="-22.606" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-22.606" x2="-2.54" y2="-22.606" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-25.146" x2="-2.54" y2="-25.654" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-25.654" x2="-1.016" y2="-25.654" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-25.654" x2="-1.016" y2="-25.146" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-25.146" x2="-2.54" y2="-25.146" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-27.686" x2="-2.54" y2="-28.194" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-28.194" x2="-1.016" y2="-28.194" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-28.194" x2="-1.016" y2="-27.686" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-27.686" x2="-2.54" y2="-27.686" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-30.226" x2="-2.54" y2="-30.734" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-30.734" x2="-1.016" y2="-30.734" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-30.734" x2="-1.016" y2="-30.226" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-30.226" x2="-2.54" y2="-30.226" width="0.254" layer="94"/>
<text x="-5.08" y="30.48" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="-34.29" size="1.778" layer="96">&gt;VALUE</text>
<pin name="-1" x="-7.62" y="27.94" visible="pin" length="short" direction="pas"/>
<pin name="-2" x="-7.62" y="25.4" visible="pin" length="short" direction="pas"/>
<pin name="-3" x="-7.62" y="22.86" visible="pin" length="short" direction="pas"/>
<pin name="-4" x="-7.62" y="20.32" visible="pin" length="short" direction="pas"/>
<pin name="-5" x="-7.62" y="17.78" visible="pin" length="short" direction="pas"/>
<pin name="-6" x="-7.62" y="15.24" visible="pin" length="short" direction="pas"/>
<pin name="-7" x="-7.62" y="12.7" visible="pin" length="short" direction="pas"/>
<pin name="-8" x="-7.62" y="10.16" visible="pin" length="short" direction="pas"/>
<pin name="-9" x="-7.62" y="7.62" visible="pin" length="short" direction="pas"/>
<pin name="-10" x="-7.62" y="5.08" visible="pin" length="short" direction="pas"/>
<pin name="-11" x="-7.62" y="2.54" visible="pin" length="short" direction="pas"/>
<pin name="-12" x="-7.62" y="0" visible="pin" length="short" direction="pas"/>
<pin name="-13" x="-7.62" y="-2.54" visible="pin" length="short" direction="pas"/>
<pin name="-14" x="-7.62" y="-5.08" visible="pin" length="short" direction="pas"/>
<pin name="-15" x="-7.62" y="-7.62" visible="pin" length="short" direction="pas"/>
<pin name="-16" x="-7.62" y="-10.16" visible="pin" length="short" direction="pas"/>
<pin name="-17" x="-7.62" y="-12.7" visible="pin" length="short" direction="pas"/>
<pin name="-18" x="-7.62" y="-15.24" visible="pin" length="short" direction="pas"/>
<pin name="-19" x="-7.62" y="-17.78" visible="pin" length="short" direction="pas"/>
<pin name="-20" x="-7.62" y="-20.32" visible="pin" length="short" direction="pas"/>
<pin name="-21" x="-7.62" y="-22.86" visible="pin" length="short" direction="pas"/>
<pin name="-22" x="-7.62" y="-25.4" visible="pin" length="short" direction="pas"/>
<pin name="-23" x="-7.62" y="-27.94" visible="pin" length="short" direction="pas"/>
<pin name="-24" x="-7.62" y="-30.48" visible="pin" length="short" direction="pas"/>
<pin name="B-24" x="-5.08" y="-30.48" visible="off" length="short" direction="pas"/>
<pin name="B-1" x="-5.08" y="27.94" visible="off" length="short" direction="pas"/>
<pin name="B-2" x="-5.08" y="25.4" visible="off" length="short" direction="pas"/>
<pin name="B-3" x="-5.08" y="22.86" visible="off" length="short" direction="pas"/>
<pin name="B-4" x="-5.08" y="20.32" visible="off" length="short" direction="pas"/>
<pin name="B-5" x="-5.08" y="17.78" visible="off" length="short" direction="pas"/>
<pin name="B-6" x="-5.08" y="15.24" visible="off" length="short" direction="pas"/>
<pin name="B-7" x="-5.08" y="12.7" visible="off" length="short" direction="pas"/>
<pin name="B-8" x="-5.08" y="10.16" visible="off" length="short" direction="pas"/>
<pin name="B-9" x="-5.08" y="7.62" visible="off" length="short" direction="pas"/>
<pin name="B-10" x="-5.08" y="5.08" visible="off" length="short" direction="pas"/>
<pin name="B-11" x="-5.08" y="2.54" visible="off" length="short" direction="pas"/>
<pin name="B-12" x="-5.08" y="0" visible="off" length="short" direction="pas"/>
<pin name="B-13" x="-5.08" y="-2.54" visible="off" length="short" direction="pas"/>
<pin name="B-14" x="-5.08" y="-5.08" visible="off" length="short" direction="pas"/>
<pin name="B-15" x="-5.08" y="-7.62" visible="off" length="short" direction="pas"/>
<pin name="B-16" x="-5.08" y="-10.16" visible="off" length="short" direction="pas"/>
<pin name="B-17" x="-5.08" y="-12.7" visible="off" length="short" direction="pas"/>
<pin name="B-18" x="-5.08" y="-15.24" visible="off" length="short" direction="pas"/>
<pin name="B-19" x="-5.08" y="-17.78" visible="off" length="short" direction="pas"/>
<pin name="B-20" x="-5.08" y="-20.32" visible="off" length="short" direction="pas"/>
<pin name="B-21" x="-5.08" y="-22.86" visible="off" length="short" direction="pas"/>
<pin name="B-22" x="-5.08" y="-25.4" visible="off" length="short" direction="pas"/>
<pin name="B-23" x="-5.08" y="-27.94" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="233-224" prefix="X" uservalue="yes">
<description>&lt;b&gt;WAGO&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="KL-24" x="0" y="0"/>
</gates>
<devices>
<device name="" package="233-224">
<connects>
<connect gate="G$1" pin="-1" pad="A1"/>
<connect gate="G$1" pin="-10" pad="A10"/>
<connect gate="G$1" pin="-11" pad="A11"/>
<connect gate="G$1" pin="-12" pad="A12"/>
<connect gate="G$1" pin="-13" pad="A13"/>
<connect gate="G$1" pin="-14" pad="A14"/>
<connect gate="G$1" pin="-15" pad="A15"/>
<connect gate="G$1" pin="-16" pad="A16"/>
<connect gate="G$1" pin="-17" pad="A17"/>
<connect gate="G$1" pin="-18" pad="A18"/>
<connect gate="G$1" pin="-19" pad="A19"/>
<connect gate="G$1" pin="-2" pad="A2"/>
<connect gate="G$1" pin="-20" pad="A20"/>
<connect gate="G$1" pin="-21" pad="A21"/>
<connect gate="G$1" pin="-22" pad="A22"/>
<connect gate="G$1" pin="-23" pad="A23"/>
<connect gate="G$1" pin="-24" pad="A24"/>
<connect gate="G$1" pin="-3" pad="A3"/>
<connect gate="G$1" pin="-4" pad="A4"/>
<connect gate="G$1" pin="-5" pad="A5"/>
<connect gate="G$1" pin="-6" pad="A6"/>
<connect gate="G$1" pin="-7" pad="A7"/>
<connect gate="G$1" pin="-8" pad="A8"/>
<connect gate="G$1" pin="-9" pad="A9"/>
<connect gate="G$1" pin="B-1" pad="B1"/>
<connect gate="G$1" pin="B-10" pad="B10"/>
<connect gate="G$1" pin="B-11" pad="B11"/>
<connect gate="G$1" pin="B-12" pad="B12"/>
<connect gate="G$1" pin="B-13" pad="B13"/>
<connect gate="G$1" pin="B-14" pad="B14"/>
<connect gate="G$1" pin="B-15" pad="B15"/>
<connect gate="G$1" pin="B-16" pad="B16"/>
<connect gate="G$1" pin="B-17" pad="B17"/>
<connect gate="G$1" pin="B-18" pad="B18"/>
<connect gate="G$1" pin="B-19" pad="B19"/>
<connect gate="G$1" pin="B-2" pad="B2"/>
<connect gate="G$1" pin="B-20" pad="B20"/>
<connect gate="G$1" pin="B-21" pad="B21"/>
<connect gate="G$1" pin="B-22" pad="B22"/>
<connect gate="G$1" pin="B-23" pad="B23"/>
<connect gate="G$1" pin="B-24" pad="B24"/>
<connect gate="G$1" pin="B-3" pad="B3"/>
<connect gate="G$1" pin="B-4" pad="B4"/>
<connect gate="G$1" pin="B-5" pad="B5"/>
<connect gate="G$1" pin="B-6" pad="B6"/>
<connect gate="G$1" pin="B-7" pad="B7"/>
<connect gate="G$1" pin="B-8" pad="B8"/>
<connect gate="G$1" pin="B-9" pad="B9"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="233-224" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="28K9070" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-garry">
<description>&lt;b&gt;Pin Header Connectors&lt;/b&gt;&lt;p&gt;
&lt;a href="www.mpe-connector.de"&gt;Menufacturer&lt;/a&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="332-10">
<description>&lt;b&gt;10 Pin - 2mm Dual Row&lt;/b&gt;&lt;p&gt;
Source: www.mpe-connector.de / garry_shortform_2012.pdf</description>
<wire x1="-4.85" y1="-1.9" x2="4.85" y2="-1.9" width="0.2032" layer="21"/>
<wire x1="4.85" y1="-1.9" x2="4.85" y2="-0.4" width="0.2032" layer="21"/>
<wire x1="4.85" y1="0.4" x2="4.85" y2="1.9" width="0.2032" layer="21"/>
<wire x1="4.85" y1="1.9" x2="-4.85" y2="1.9" width="0.2032" layer="21"/>
<wire x1="-4.85" y1="1.9" x2="-4.85" y2="0.4" width="0.2032" layer="21"/>
<wire x1="-4.85" y1="-0.4" x2="-4.85" y2="-1.9" width="0.2032" layer="21"/>
<wire x1="-4.85" y1="0.4" x2="-4.85" y2="-0.4" width="0.2032" layer="21" curve="-129.185"/>
<wire x1="4.85" y1="-0.4" x2="4.85" y2="0.4" width="0.2032" layer="21" curve="-129.185"/>
<pad name="1" x="-4" y="-1" drill="0.9" diameter="1.27"/>
<pad name="2" x="-4" y="1" drill="0.9" diameter="1.27"/>
<pad name="3" x="-2" y="-1" drill="0.9" diameter="1.27"/>
<pad name="4" x="-2" y="1" drill="0.9" diameter="1.27"/>
<pad name="5" x="0" y="-1" drill="0.9" diameter="1.27"/>
<pad name="6" x="0" y="1" drill="0.9" diameter="1.27"/>
<pad name="7" x="2" y="-1" drill="0.9" diameter="1.27"/>
<pad name="8" x="2" y="1" drill="0.9" diameter="1.27"/>
<pad name="9" x="4" y="-1" drill="0.9" diameter="1.27"/>
<pad name="10" x="4" y="1" drill="0.9" diameter="1.27"/>
<text x="-4.65" y="-1.75" size="0.3048" layer="21" font="vector">1</text>
<rectangle x1="-4.25" y1="-1.25" x2="-3.75" y2="-0.75" layer="51"/>
<rectangle x1="-4.25" y1="0.75" x2="-3.75" y2="1.25" layer="51"/>
<rectangle x1="-2.25" y1="-1.25" x2="-1.75" y2="-0.75" layer="51"/>
<rectangle x1="-2.25" y1="0.75" x2="-1.75" y2="1.25" layer="51"/>
<rectangle x1="-0.25" y1="-1.25" x2="0.25" y2="-0.75" layer="51"/>
<rectangle x1="-0.25" y1="0.75" x2="0.25" y2="1.25" layer="51"/>
<rectangle x1="1.75" y1="-1.25" x2="2.25" y2="-0.75" layer="51"/>
<rectangle x1="1.75" y1="0.75" x2="2.25" y2="1.25" layer="51"/>
<rectangle x1="3.75" y1="-1.25" x2="4.25" y2="-0.75" layer="51"/>
<rectangle x1="3.75" y1="0.75" x2="4.25" y2="1.25" layer="51"/>
<text x="-4.62" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="1.73" y="2.54" size="1.27" layer="27">&gt;VALUE</text>
<wire x1="-4" y1="1" x2="-4" y2="-6" width="0.55" layer="51"/>
<wire x1="-2" y1="1" x2="-2" y2="-6" width="0.55" layer="51"/>
<wire x1="0" y1="1" x2="0" y2="-6" width="0.55" layer="51"/>
<wire x1="2" y1="1" x2="2" y2="-6" width="0.55" layer="51"/>
<wire x1="4" y1="1" x2="4" y2="-6" width="0.55" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="MV">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<text x="-0.762" y="1.397" size="1.778" layer="96">&gt;VALUE</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="M">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="332-10" prefix="X">
<description>&lt;b&gt;10 Pin - 2mm Dual Row&lt;/b&gt;&lt;p&gt;
Source: www.mpe-connector.de / garry_shortform_2012.pdf</description>
<gates>
<gate name="-1" symbol="MV" x="-10.16" y="5.08" addlevel="always"/>
<gate name="-2" symbol="MV" x="10.16" y="5.08" addlevel="always"/>
<gate name="-3" symbol="M" x="-10.16" y="2.54" addlevel="always"/>
<gate name="-4" symbol="M" x="10.16" y="2.54" addlevel="always"/>
<gate name="-5" symbol="M" x="-10.16" y="0" addlevel="always"/>
<gate name="-6" symbol="M" x="10.16" y="0" addlevel="always"/>
<gate name="-7" symbol="M" x="-10.16" y="-2.54" addlevel="always"/>
<gate name="-8" symbol="M" x="10.16" y="-2.54" addlevel="always"/>
<gate name="-9" symbol="M" x="-10.16" y="-5.08" addlevel="always"/>
<gate name="-10" symbol="M" x="10.16" y="-5.08" addlevel="always"/>
</gates>
<devices>
<device name="" package="332-10">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-10" pin="S" pad="10"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
<connect gate="-8" pin="S" pad="8"/>
<connect gate="-9" pin="S" pad="9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="X1" library="con-wago255" deviceset="233-224" device=""/>
<part name="X2" library="con-garry" deviceset="332-10" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="X1" gate="G$1" x="139.7" y="45.72"/>
<instance part="X2" gate="-1" x="85.09" y="8.89"/>
<instance part="X2" gate="-2" x="105.41" y="8.89"/>
<instance part="X2" gate="-3" x="85.09" y="6.35"/>
<instance part="X2" gate="-4" x="105.41" y="6.35"/>
<instance part="X2" gate="-5" x="85.09" y="3.81"/>
<instance part="X2" gate="-6" x="105.41" y="3.81"/>
<instance part="X2" gate="-7" x="85.09" y="1.27"/>
<instance part="X2" gate="-8" x="105.41" y="1.27"/>
<instance part="X2" gate="-9" x="85.09" y="-1.27"/>
<instance part="X2" gate="-10" x="105.41" y="-1.27"/>
</instances>
<busses>
</busses>
<nets>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
