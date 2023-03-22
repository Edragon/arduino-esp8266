// change these values to match your network
char ssid[] = "111";       //  your network SSID (name)
char pass[] = "electrodragon";          //  your network password
 

String header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
 
String html_1 = R"=====(
<!DOCTYPE html>
<html>


 <head>
 
 <meta name='viewport' content='width=device-width, initial-scale=1.0'/>
 <meta charset='utf-8'>
 <meta http-equiv='refresh' content='5'>

 
 <style>
   body {font-size:100%;} 
   #main {display: table; margin: auto;  padding: 0 10px 0 10px; } 
   h2 {text-align:center; } 
   p { text-align:center; }
 </style>

 
   <title>Test ...</title>
 </head>

 
 <body>
   <div id='main'>
     <h2>Auto Update Example Using HTML Electrodragon</h2>
     
     <div id='count'> 
       <p>Count = %count%</p>
     </div>
     
   </div> 
   
 </body>
</html>
)====="; 