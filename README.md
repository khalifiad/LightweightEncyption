# Sistem Komunikasi LoRa dengan Enkripsi

Proyek ini mengimplementasikan sistem komunikasi nirkabel menggunakan LoRa (Long Range) dengan tiga algoritma enkripsi berbeda: PRESENT, ChaCha20, dan Speck. Sistem terdiri dari transmitter (pengirim) dan receiver (penerima) yang berkomunikasi secara aman.

## 🎯 Pengenalan
### Apa itu LoRa?
LoRa (Long Range) adalah teknologi komunikasi nirkabel jarak jauh dengan konsumsi daya rendah. Cocok untuk aplikasi IoT yang membutuhkan jangkauan komunikasi hingga beberapa kilometer.

### Mengapa Enkripsi Diperlukan?
Komunikasi nirkabel rentan terhadap penyadapan. Enkripsi melindungi data yang dikirim melalui udara agar hanya penerima yang memiliki kunci yang tepat yang dapat membaca pesan.

## 🔐 Algoritma Enkripsi

### 1. PRESENT (Lightweight Block Cipher)
- **Jenis:** Block cipher
- **Ukuran Blok:** 64 bit
- **Ukuran Kunci:** 80 bit (10 byte)
- **Jumlah Round:** 31

**Cara Kerja:**
```
Plaintext (64-bit) → AddRoundKey → SBox → PLayer (31x) → Ciphertext
```

### 2. ChaCha20 (Stream Cipher)
- **Jenis:** Stream cipher
- **Ukuran Kunci:** 256 bit (32 byte)
- **Nonce:** 96 bit (12 byte)
- **Jumlah Round:** 20 (10 double-rounds)
- 
**Cara Kerja:**
```
Key + Nonce + Counter → ChaCha20 Block → Keystream
Plaintext XOR Keystream → Ciphertext
```

### 3. Speck (Lightweight Block Cipher)
- **Jenis:** Block cipher
- **Ukuran Blok:** 64 bit
- **Ukuran Kunci:** 96 bit (3 × 32-bit words)
- **Jumlah Round:** 26

**Cara Kerja:**
```
Plaintext → Round Function (ROR, ADD, ROL, XOR) 26x → Ciphertext
```
