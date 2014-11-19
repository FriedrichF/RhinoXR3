package robotik7;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class Ausgabe {
	FileWriter writer;
	File file;
	
	public void openFile(String name){
		file = new File(name);
		try{
			writer = new FileWriter(file, true);
		}catch(IOException e){
			e.printStackTrace();
		}
	}
	
	public void writeToFile(String text){
		try {
			writer.write(text);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void closeFile(){
		try {
			writer.flush();
			writer.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}
