## TODO / Planned

- [ ] NAPPEIHIN "LONGPRESS"-TOIMINTO

- [ ] HISTORIARUUTU (24H Alkuun muistitaulukosta)
        - [ ] Myös arvojen aaltoilu nähtävä keskiarvojen lisäksi, että releronklaus saadaan parametreja viilaamalla minimiin

- [ ] WS2812 - statusvalot (ja virransäästö näyttöön?)
        - [ ] Erilaisia värikoodauksia erroreille ja tavoitearvoissa pysymiselle (ehkä vähän sellain pidemmältäkin jaksolta?)
                - [ ] Esim. <90% viim. vrk tavoitearvossa vihreä (ja maks. poikkeama pieni) / yli 60% keltainen / yli 30% oranssi / <30% (ja/tai maks. poikkeama iso) punainen
                - [ ] Esim. Alle 12 vaihteen vaihtoa päivässä = vihreä / alle 24 = keltainen / alle 48 = oranssi / yli 48 = punainen

- [ ] RELELOGIIKAN TARKISTUS (onko nykyinen vikasietoinen?)
	- [ ] Mitä jos arduinosta katkeaa virta?
	- [ ] Mitä jos jokin rele hirttää kiinni?
	- [ ] Entäs jos relekortista katkeaa virta?

- [ ] GRAAFINEN ESITYSMUOTO HISTORIALLE (Custom characters)

- [ ] EEPROMILLE JÄRKEVÄ FORMAATTI (kts. seuraava)
	- [ ] Tilaa asetuksille (ja grafiikkaelementeille?)
	- [ ] Varaus myös toiseelle BME280-anturille
	- [ ] (Varaus volttiarvoille /) virheilmoituksille
    		- [ ] Vikakoodeja ehkä vain yksi tuntiin logiin tai jonkinlainen koodaus, että mikä koodin aiheutti että kaikki mahdolliset saa 1-2 tavuun
	- [ ] Ehkä jopa laskee releiden kulumista ja ilmoittaa kun alkaa olla aika vaihtaa??? (Jos siis saa järkevästi jotenkin ilman, että tarvii laskea ja kirjata joka naksu)
	- [ ] Ehkä joku koodaus että 5-10v tulee kevyelläkin täyteen ja kirjaa päivä/viikkotasolla kevyt/keskiraskas/raskas/äärimmäinen
	- [ ] Tiedostojärjestelmän formaattiversio kirjataan ylös ja kysyttäessä formatoidaan, jos vanha formaatti ei yhteensopiva/tunnistettu
    		- [ ] Hallittu siirtymä uuteen formaattiin!!! (vaikka jollain apuohjelmalla tai Serial-yhteyden avulla toteutetulla siirtoformatoinnilla)

 - [ ] PIDEMMÄN AJAN HISTORIA EEPROMILLE (Rengaspuskuri)
	- [ ] Mahdollisuus tyhjentämiseen ja siirtoon koneelle?
		- [ ] Älä kuitenkaan nollaa reledataa samalla

- [ ] VOLTTIMITTARI (RELEIDEN TOIMINNAN SEURANTA)
	- [ ] Seuraa, että vaiteet vaihtuu oikein ja poistaa tarvittaessa rikkinäisen releen käytöstä & tallentaa vikakoodin.

- [ ] Bluetooth-moduuli toisen BM280:n tilalle tai rinnalle


